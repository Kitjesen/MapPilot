# Native endpoint targets are owned by src/nav/cpp/CMakeLists.txt.

find_package(iceoryx_binding_c QUIET)
find_package(CycloneDDS REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(
  LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER
  "Build OctoPlanner-backed endpoint targets"
  ON)

set(_LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../..")
set(_LINGTU_IDL "${_LINGTU_ROOT}/src/message/idl/messages.idl")
set(_NAV_CPP_DIR "${_LINGTU_ROOT}/src/nav/cpp")
set(_NAV_COMMANDS_CPP_DIR "${_NAV_CPP_DIR}/client")
set(_NAV_TESTS_DIR "${_NAV_CPP_DIR}/tests")
set(_NAV_ENDPOINT_TESTS_DIR "${_NAV_TESTS_DIR}/endpoint")
set(_EXPLORE_CPP_DIR "${_LINGTU_ROOT}/src/explore/cpp")
set(_PERCEPTION_INSPECTION_DIR "${_LINGTU_ROOT}/src/perception/inspection")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${_NAV_CPP_DIR}/planning/global/octoplanner")

include("${_NAV_CPP_DIR}/cmake/NavCoreTargets.cmake")
set(_NAV_CORE_DIR "${LINGTU_NAV_CPP_ROOT}")
set(_NAV_KERNEL_DIR "${LINGTU_NAV_KERNEL_DIR}")
set(_EP_DIRS
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/nav"
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
if(NOT TARGET lingtu_dds_contracts)
  add_subdirectory(
    "${_LINGTU_ROOT}/src/message/cpp"
    "${CMAKE_BINARY_DIR}/message_cpp")
endif()
lingtu_add_dds_c_messages(lingtu_dds_messages "${_LINGTU_IDL}")

# Windows resolves dependent DLLs from the executable directory before PATH.
# Keep every endpoint-owned executable and shared library in one configuration
# directory so tests, tools, and Product binaries share the same app-local
# runtime closure. External subprojects above retain their own output layout.
if(WIN32)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()

function(lingtu_sources_exist out_var)
  set(_ok TRUE)
  foreach(_path IN LISTS ARGN)
    if(NOT EXISTS "${_path}")
      set(_ok FALSE)
    endif()
  endforeach()
  set("${out_var}" "${_ok}" PARENT_SCOPE)
endfunction()

function(lingtu_endpoint_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

function(lingtu_endpoint_enable_assertions target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /UNDEBUG)
  else()
    target_compile_options(${target} PRIVATE -UNDEBUG)
  endif()
endfunction()

function(lingtu_endpoint_bundle_cyclonedds target)
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

if(LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER
   AND EXISTS "${_OCTOPLANNER3D_RUNTIME_DIR}/CMakeLists.txt")
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
  lingtu_dds_contracts
  Threads::Threads)

target_compile_definitions(lingtu_nav_client PRIVATE
  LINGTU_NAV_CLIENT_BUILD)

lingtu_endpoint_enable_warnings(lingtu_nav_client)
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
  lingtu_dds_contracts
  Threads::Threads)

target_compile_definitions(lingtu_inspection_evidence_bridge PRIVATE
  LINGTU_INSPECTION_EVIDENCE_BRIDGE_BUILD)

lingtu_endpoint_enable_warnings(lingtu_inspection_evidence_bridge)
set_target_properties(lingtu_inspection_evidence_bridge PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
  INSTALL_RPATH "$ORIGIN")

if(LINGTU_NAV_CPP_BUILD_TESTS)
add_executable(test_nav_client
  "${_NAV_TESTS_DIR}/test_client.cpp"
  nav/command/ingress.cpp)
target_include_directories(test_nav_client PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}"
  "${_LINGTU_ROOT}/src"
  "${_NAV_COMMANDS_CPP_DIR}"
  ${_EP_DIRS})
target_link_libraries(test_nav_client PRIVATE
  lingtu_nav_client
  lingtu_dds_messages
  CycloneDDS::ddsc
  lingtu_dds_contracts
  Threads::Threads)
lingtu_endpoint_enable_warnings(test_nav_client)
set(_NAV_CLIENT_TEST_CASES
  command_roundtrip
  domain_contract
  goal_status
  path_telemetry
  plan_preview
  map_scene
  exploration_commands
  exploration_events
  inspection_commands
  operator_identity
  operator_receipts
  estop_bypass
  ack_replay
  source_stamp
  clock_anchor
  delayed_clock_ack
  concurrent_clients
  clock_rollback)
foreach(_NAV_CLIENT_TEST_CASE IN LISTS _NAV_CLIENT_TEST_CASES)
  if(_NAV_CLIENT_TEST_CASE STREQUAL "command_roundtrip")
    set(_NAV_CLIENT_TEST_NAME test_nav_client)
  else()
    set(_NAV_CLIENT_TEST_NAME "test_nav_client_${_NAV_CLIENT_TEST_CASE}")
  endif()
  add_test(
    NAME "${_NAV_CLIENT_TEST_NAME}"
    COMMAND test_nav_client --case "${_NAV_CLIENT_TEST_CASE}")
  set_tests_properties("${_NAV_CLIENT_TEST_NAME}" PROPERTIES TIMEOUT 10)
endforeach()
endif()

if(_HAS_OCTOPLANNER3D_RUNTIME)
  add_executable(navd
    nav/input/active/octomap.cpp
    nav/input/active/occupancy.cpp
    nav/input/active/declared.cpp
    nav/main.cpp
    nav/runtime/loop.cpp
    nav/input/pose.cpp
    nav/input/map.cpp
    nav/input/health.cpp
    nav/input/planner.cpp
    nav/runtime/inspection/inspection_command_coordinator.cpp
    nav/runtime/inspection/inspection_runtime_controller.cpp
    nav/runtime/goal/blockage.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp
    nav/command/goal.cpp
    nav/command/terminal.cpp
    nav/dds/runtime.cpp
    nav/runtime/config/parse.cpp
    nav/runtime/config/build.cpp
    nav/input/gate.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/control/guard.cpp
    nav/control/authority.cpp
    nav/control/operator.cpp
    nav/status/control_loop_health.cpp
    nav/status/nav_status_writer.cpp
    nav/status/nav_status_publisher.cpp
    nav/status/nav_status_endpoint_adapter.cpp
    nav/status/navigation_goal_status_outbox.cpp
    nav/status/goal_terminal_status_delivery.cpp
    nav/status/goal_terminal_transaction.cpp
    nav/runtime/navigation.cpp
    nav/status/navigation_state.cpp
    nav/status/status_snapshot_file_writer.cpp
    nav/status/inspection_status_file_writer.cpp
    nav/status/active_inspection_map_cache.cpp
    nav/command/cancel.cpp
    nav/command/ingress.cpp
    nav/safety/geofence.cpp
    nav/safety/stop.cpp
    nav/control/admission.cpp
    nav/control/final.cpp
    nav/control/teleop.cpp
    nav/control/autonomy.cpp
    nav/input/obstacle.cpp
    nav/safety/command.cpp
  )

  target_include_directories(navd PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    "${_EXPLORE_CPP_DIR}"
    "${_OCTOPLANNER3D_RUNTIME_DIR}"
    ${_EP_DIRS})

  target_link_libraries(navd PRIVATE
    lingtu_inspection_core
    lingtu_maps
    lingtu_nav_far
    lingtu_nav_navigation
    lingtu_nav_rolling_segment
    lingtu_nav_velocity_smoother
    lingtu_dds_messages
    octoplanner3d_runtime
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(navd PRIVATE OpenMP::OpenMP_CXX)
  endif()

  lingtu_endpoint_enable_warnings(navd)
  lingtu_nav_enable_performance(navd)
  set_target_properties(navd PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
elseif(LINGTU_NAV_ENDPOINT_BUILD_OCTOPLANNER)
  message(FATAL_ERROR "Cannot build required navd target: OctoPlanner3D runtime is missing")
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp")
  add_executable(test_global_plan_task
    "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp"
    nav/runtime/goal/task.cpp)
  target_include_directories(test_global_plan_task PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_global_plan_task PRIVATE
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_global_plan_task)
  add_test(NAME test_global_plan_task COMMAND test_global_plan_task)
endif()

if(NOT TARGET test_bounded_goal_replan_controller
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_bounded_goal_replan_controller.cpp")
  add_executable(test_bounded_goal_replan_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_bounded_goal_replan_controller.cpp"
    nav/runtime/goal/retry.cpp)
  target_include_directories(test_bounded_goal_replan_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_bounded_goal_replan_controller)
  add_test(NAME test_bounded_goal_replan_controller COMMAND test_bounded_goal_replan_controller)
endif()

if(NOT TARGET test_active_path_blockage_policy
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_blockage_policy.cpp"
    AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/nav/runtime/goal/blockage.cpp")
  add_executable(test_active_path_blockage_policy
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_blockage_policy.cpp"
    nav/runtime/goal/blockage.cpp)
  target_include_directories(test_active_path_blockage_policy PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_active_path_blockage_policy)
  add_test(NAME test_active_path_blockage_policy
           COMMAND test_active_path_blockage_policy)
endif()

if(NOT TARGET test_goal_plan_controller
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_controller.cpp")
  add_executable(test_goal_plan_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_controller.cpp"
    nav/runtime/goal/task.cpp
    nav/runtime/goal/plan.cpp)
  target_include_directories(test_goal_plan_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_controller PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_goal_plan_controller)
  add_test(NAME test_goal_plan_controller COMMAND test_goal_plan_controller)
endif()

if(NOT TARGET test_goal_replan_runtime_coordinator
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_replan_runtime_coordinator.cpp")
  add_executable(test_goal_replan_runtime_coordinator
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_replan_runtime_coordinator.cpp"
    nav/command/cancel.cpp
    nav/command/ingress.cpp
    nav/safety/stop.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp)
  target_include_directories(test_goal_replan_runtime_coordinator PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_replan_runtime_coordinator PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_goal_replan_runtime_coordinator)
  add_test(NAME test_goal_replan_runtime_coordinator
           COMMAND test_goal_replan_runtime_coordinator)
endif()

if(NOT TARGET test_active_path_replan_cycle
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_replan_cycle.cpp")
  add_executable(test_active_path_replan_cycle
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_replan_cycle.cpp"
    nav/runtime/goal/blockage.cpp
    nav/control/autonomy.cpp
    nav/control/final.cpp
    nav/safety/stop.cpp
    nav/safety/command.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp)
  target_include_directories(test_active_path_replan_cycle PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_active_path_replan_cycle PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_active_path_replan_cycle)
  add_test(NAME test_active_path_replan_cycle
           COMMAND test_active_path_replan_cycle)
endif()

if(NOT TARGET test_goal_terminal_transaction
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_terminal_transaction.cpp")
  add_executable(test_goal_terminal_transaction
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_terminal_transaction.cpp"
    nav/status/goal_terminal_status_delivery.cpp
    nav/status/goal_terminal_transaction.cpp
    nav/status/navigation_goal_status_outbox.cpp
    nav/safety/stop.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp)
  target_include_directories(test_goal_terminal_transaction PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_goal_terminal_transaction PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_goal_terminal_transaction)
  add_test(NAME test_goal_terminal_transaction
           COMMAND test_goal_terminal_transaction)
endif()

if(NOT TARGET test_navigation_runtime_controller
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_runtime_controller.cpp")
  add_executable(test_navigation_runtime_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_runtime_controller.cpp"
    nav/runtime/navigation.cpp
    nav/status/goal_terminal_status_delivery.cpp
    nav/status/goal_terminal_transaction.cpp
    nav/status/navigation_goal_status_outbox.cpp
    nav/safety/stop.cpp
    nav/runtime/goal/task.cpp
    nav/runtime/goal/retry.cpp
    nav/runtime/goal/plan.cpp
    nav/runtime/goal/runtime.cpp)
  target_include_directories(test_navigation_runtime_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_navigation_runtime_controller PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_navigation_runtime_controller)
  add_test(NAME test_navigation_runtime_controller
           COMMAND test_navigation_runtime_controller)
endif()

if(NOT TARGET test_goal_terminal_ingress_policy
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_terminal_ingress_policy.cpp")
  add_executable(test_goal_terminal_ingress_policy
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_terminal_ingress_policy.cpp"
    nav/command/terminal.cpp)
  target_include_directories(test_goal_terminal_ingress_policy PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_goal_terminal_ingress_policy)
  add_test(NAME test_goal_terminal_ingress_policy
           COMMAND test_goal_terminal_ingress_policy)
endif()

if(NOT TARGET test_goal_plan_completion
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp")
  add_executable(test_goal_plan_completion
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp"
    nav/runtime/goal/task.cpp
    nav/runtime/goal/plan.cpp)
  target_include_directories(test_goal_plan_completion PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_completion PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_goal_plan_completion)
  add_test(NAME test_goal_plan_completion COMMAND test_goal_plan_completion)
endif()

if(NOT TARGET test_rolling_map_segment_executor
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_map_segment_executor.cpp")
  add_executable(test_rolling_map_segment_executor
    "${_NAV_ENDPOINT_TESTS_DIR}/test_rolling_map_segment_executor.cpp")
  target_link_libraries(test_rolling_map_segment_executor PRIVATE
    lingtu_nav_rolling_segment)
  lingtu_endpoint_enable_warnings(test_rolling_map_segment_executor)
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
  lingtu_endpoint_enable_warnings(test_rolling_segment_lifecycle)
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
  lingtu_endpoint_enable_warnings(test_rolling_segment_effect_coordinator)
  add_test(
    NAME test_rolling_segment_effect_coordinator
    COMMAND test_rolling_segment_effect_coordinator)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer_basics.cpp")
  add_executable(test_motion_layer_basics
    "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer_basics.cpp"
    nav/input/obstacle.cpp)

  target_include_directories(test_motion_layer_basics PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")

  lingtu_endpoint_enable_warnings(test_motion_layer_basics)
  add_test(NAME test_motion_layer_basics COMMAND test_motion_layer_basics)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer.cpp")
  add_executable(test_motion_layer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_layer.cpp"
    nav/input/obstacle.cpp)

  target_include_directories(test_motion_layer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")

  lingtu_endpoint_enable_warnings(test_motion_layer)
  add_test(NAME test_motion_layer COMMAND test_motion_layer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_messages.cpp")
  add_executable(test_nav_endpoint_messages
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_messages.cpp"
    nav/dds/codec.cpp
    nav/command/goal.cpp
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/input/planner.cpp
    nav/input/obstacle.cpp
    nav/safety/command.cpp
  )

  target_include_directories(test_nav_endpoint_messages PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")

  target_link_libraries(test_nav_endpoint_messages PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    Threads::Threads)

  lingtu_endpoint_enable_warnings(test_nav_endpoint_messages)
  add_test(NAME test_nav_endpoint_messages COMMAND test_nav_endpoint_messages)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_final_velocity_writer.cpp")
  add_executable(test_final_velocity_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_final_velocity_writer.cpp"
    nav/dds/runtime.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/command/goal.cpp
    nav/input/frame.cpp)
  target_include_directories(test_final_velocity_writer PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_final_velocity_writer PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_final_velocity_writer)
  add_test(
    NAME test_final_velocity_writer
    COMMAND test_final_velocity_writer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_dds.cpp")
  add_executable(test_inspection_evidence_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_evidence_dds.cpp"
    nav/dds/runtime.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/command/goal.cpp
    nav/input/frame.cpp)
  target_include_directories(test_inspection_evidence_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_evidence_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_inspection_evidence_dds)
  add_test(
    NAME test_inspection_evidence_dds
    COMMAND test_inspection_evidence_dds)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_goal_status_dds.cpp")
  add_executable(test_navigation_goal_status_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_goal_status_dds.cpp"
    nav/dds/runtime.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/command/goal.cpp
    nav/input/frame.cpp)
  target_include_directories(test_navigation_goal_status_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_navigation_goal_status_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_navigation_goal_status_dds)
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
    lingtu_dds_contracts
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_inspection_evidence_bridge)
  add_test(
    NAME test_inspection_evidence_bridge
    COMMAND test_inspection_evidence_bridge)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_input_gate.cpp")
  add_executable(test_input_gate "${_NAV_ENDPOINT_TESTS_DIR}/test_input_gate.cpp" nav/input/gate.cpp)
  target_include_directories(test_input_gate PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_input_gate)
  add_test(NAME test_input_gate COMMAND test_input_gate)
endif()
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_geofence.cpp")
  add_executable(test_geofence
    "${_NAV_ENDPOINT_TESTS_DIR}/test_geofence.cpp"
    nav/safety/geofence.cpp)
  target_include_directories(test_geofence PRIVATE ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_geofence)
  add_test(NAME test_geofence COMMAND test_geofence)
endif()
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_input_projector.cpp")
  add_executable(test_input_projector
    "${_NAV_ENDPOINT_TESTS_DIR}/test_input_projector.cpp"
    nav/input/pose.cpp
    nav/input/map.cpp
    nav/input/health.cpp
    nav/dds/codec.cpp
    nav/command/goal.cpp
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/input/planner.cpp
    nav/input/gate.cpp
    nav/input/obstacle.cpp)
  target_include_directories(test_input_projector PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_input_projector PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_input_projector)
  add_test(
    NAME test_input_projector
    COMMAND test_input_projector)
endif()


if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ack_journal.cpp")
  add_executable(test_command_ack_journal
    "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ack_journal.cpp"
    nav/command/ingress.cpp)
  target_include_directories(test_command_ack_journal PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")
  lingtu_endpoint_enable_warnings(test_command_ack_journal)
  add_test(NAME test_command_ack_journal COMMAND test_command_ack_journal)
endif()

if(NOT TARGET test_command_ingress_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ingress_controller.cpp")
  add_executable(test_command_ingress_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_command_ingress_controller.cpp"
    nav/command/ingress.cpp)
  target_include_directories(test_command_ingress_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")
  lingtu_endpoint_enable_warnings(test_command_ingress_controller)
  add_test(
    NAME test_command_ingress_controller
    COMMAND test_command_ingress_controller)
endif()

if(NOT TARGET test_motion_stop_barrier
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_stop_barrier.cpp")
  add_executable(test_motion_stop_barrier
    "${_NAV_ENDPOINT_TESTS_DIR}/test_motion_stop_barrier.cpp"
    nav/safety/stop.cpp)
  target_include_directories(test_motion_stop_barrier PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_motion_stop_barrier)
  add_test(
    NAME test_motion_stop_barrier
    COMMAND test_motion_stop_barrier)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp")
  add_executable(test_operator_motion_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp"
    nav/dds/runtime.cpp
    nav/dds/codec.cpp
    nav/dds/frame.cpp
    nav/command/goal.cpp
    nav/input/frame.cpp)
  target_include_directories(test_operator_motion_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_operator_motion_dds PRIVATE
    lingtu_inspection_core
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_operator_motion_dds)
  add_test(
    NAME test_operator_motion_dds
    COMMAND test_operator_motion_dds)
endif()

if(WIN32)
  # Production launchers provide one host boot identity shared by all native
  # processes. Give DDS integration tests the same contract so plain CTest is
  # portable and individual test executables do not invent Linux /proc fallbacks.
  foreach(_boot_identity_test IN ITEMS
      test_final_velocity_writer
      test_inspection_evidence_dds
      test_navigation_goal_status_dds
      test_operator_motion_dds)
    if(TEST ${_boot_identity_test})
      set_tests_properties(${_boot_identity_test} PROPERTIES
        ENVIRONMENT "LINGTU_HOST_BOOT_ID=windows-ctest-host")
    endif()
  endforeach()
endif()
if(NOT TARGET test_operator_motion_authority
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_authority.cpp")
  add_executable(test_operator_motion_authority
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_authority.cpp"
    nav/control/operator.cpp
    nav/control/authority.cpp)
  target_include_directories(test_operator_motion_authority PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_operator_motion_authority)
  add_test(
    NAME test_operator_motion_authority
    COMMAND test_operator_motion_authority)
endif()

if(NOT TARGET test_operator_motion_output_evidence
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_output_evidence.cpp")
  add_executable(test_operator_motion_output_evidence
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_output_evidence.cpp"
    nav/control/operator.cpp)
  target_include_directories(test_operator_motion_output_evidence PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_operator_motion_output_evidence)
  add_test(
    NAME test_operator_motion_output_evidence
    COMMAND test_operator_motion_output_evidence)
endif()
if(NOT TARGET test_control_loop_health
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/nav/status/control_loop_health.cpp")
  add_executable(test_control_loop_health
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    nav/status/control_loop_health.cpp)
  target_include_directories(test_control_loop_health PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_control_loop_health)
  add_test(
    NAME test_control_loop_health
    COMMAND test_control_loop_health)
endif()
if(NOT TARGET test_control_loop_runtime_guard
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_runtime_guard.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_LIST_DIR}/nav/control/guard.cpp")
  add_executable(test_control_loop_runtime_guard
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_runtime_guard.cpp"
    nav/control/guard.cpp)
  target_include_directories(test_control_loop_runtime_guard PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_control_loop_runtime_guard)
  add_test(
    NAME test_control_loop_runtime_guard
    COMMAND test_control_loop_runtime_guard)
endif()
if(NOT TARGET test_nav_status_publisher
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_publisher.cpp")
  add_executable(test_nav_status_publisher
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_publisher.cpp"
    nav/status/nav_status_publisher.cpp
    nav/status/nav_status_writer.cpp
    nav/status/status_snapshot_file_writer.cpp)
  target_include_directories(test_nav_status_publisher PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_nav_status_publisher PRIVATE
    lingtu_nav_local_planner
    lingtu_nav_follower
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_nav_status_publisher)
  add_test(
    NAME test_nav_status_publisher
    COMMAND test_nav_status_publisher)
endif()
if(NOT TARGET test_navigation_state
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_state.cpp")
  add_executable(test_navigation_state
    "${_NAV_ENDPOINT_TESTS_DIR}/test_navigation_state.cpp"
    nav/status/navigation_state.cpp)
  target_include_directories(test_navigation_state PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_navigation_state)
  add_test(
    NAME test_navigation_state
    COMMAND test_navigation_state)
endif()

if(NOT TARGET test_inspection_command_coordinator
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_command_coordinator.cpp")
  add_executable(test_inspection_command_coordinator
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_command_coordinator.cpp"
    nav/runtime/inspection/inspection_command_coordinator.cpp)
  target_include_directories(test_inspection_command_coordinator PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${LINGTU_NAV_INCLUDE_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_command_coordinator PRIVATE
    lingtu_inspection_core)
  lingtu_endpoint_enable_warnings(test_inspection_command_coordinator)
  add_test(
    NAME test_inspection_command_coordinator
    COMMAND test_inspection_command_coordinator)
endif()

if(NOT TARGET test_inspection_runtime_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_runtime_controller.cpp")
  add_executable(test_inspection_runtime_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_runtime_controller.cpp"
    nav/runtime/inspection/inspection_runtime_controller.cpp)
  target_include_directories(test_inspection_runtime_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_runtime_controller PRIVATE
    lingtu_inspection_core)
  lingtu_endpoint_enable_warnings(test_inspection_runtime_controller)
  add_test(
    NAME test_inspection_runtime_controller
    COMMAND test_inspection_runtime_controller)
endif()



if(NOT TARGET test_teleop_admission_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_admission_controller.cpp")
  add_executable(test_teleop_admission_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_admission_controller.cpp"
    nav/control/admission.cpp)
  target_include_directories(test_teleop_admission_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src"
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_teleop_admission_controller)
  add_test(
    NAME test_teleop_admission_controller
    COMMAND test_teleop_admission_controller)
endif()

if(NOT TARGET test_teleop_tick_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_tick_controller.cpp")
  add_executable(test_teleop_tick_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_tick_controller.cpp"
    nav/control/teleop.cpp
    nav/control/final.cpp
    nav/safety/command.cpp)
  target_include_directories(test_teleop_tick_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    "${_EXPLORE_CPP_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_teleop_tick_controller PRIVATE
    lingtu_nav_navigation)
  lingtu_endpoint_enable_warnings(test_teleop_tick_controller)
  add_test(
    NAME test_teleop_tick_controller
    COMMAND test_teleop_tick_controller)
endif()

if(NOT TARGET test_autonomy_tick_controller
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_autonomy_tick_controller.cpp")
  add_executable(test_autonomy_tick_controller
    "${_NAV_ENDPOINT_TESTS_DIR}/test_autonomy_tick_controller.cpp"
    nav/control/autonomy.cpp
    nav/control/final.cpp
    nav/safety/command.cpp)
  target_include_directories(test_autonomy_tick_controller PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_NAV_CORE_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_autonomy_tick_controller PRIVATE
    lingtu_nav_navigation)
  lingtu_endpoint_enable_warnings(test_autonomy_tick_controller)
  add_test(
    NAME test_autonomy_tick_controller
    COMMAND test_autonomy_tick_controller)
endif()

if(NOT TARGET test_active_inspection_map_cache
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_active_inspection_map_cache.cpp")
  add_executable(test_active_inspection_map_cache
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_inspection_map_cache.cpp"
    nav/status/active_inspection_map_cache.cpp)
  target_include_directories(test_active_inspection_map_cache PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS})
  target_link_libraries(test_active_inspection_map_cache PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_active_inspection_map_cache)
  add_test(
    NAME test_active_inspection_map_cache
    COMMAND test_active_inspection_map_cache)
endif()

if(NOT TARGET test_inspection_status_file_writer
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp")
  add_executable(test_inspection_status_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp"
    nav/status/inspection_status_file_writer.cpp
    nav/status/status_snapshot_file_writer.cpp)
  target_include_directories(test_inspection_status_file_writer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_status_file_writer PRIVATE
    lingtu_inspection_core
    Threads::Threads)
  lingtu_endpoint_enable_warnings(test_inspection_status_file_writer)
  add_test(
    NAME test_inspection_status_file_writer
    COMMAND test_inspection_status_file_writer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_stop_confirmation.cpp")
  add_executable(test_stop_confirmation
    "${_NAV_ENDPOINT_TESTS_DIR}/test_stop_confirmation.cpp"
    nav/safety/stop.cpp)
  target_include_directories(test_stop_confirmation PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  lingtu_endpoint_enable_warnings(test_stop_confirmation)
  add_test(NAME test_stop_confirmation COMMAND test_stop_confirmation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp")
  add_executable(test_control_authority
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp"
    nav/control/authority.cpp
    nav/safety/stop.cpp)
  target_include_directories(test_control_authority PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_control_authority)
  add_test(NAME test_control_authority COMMAND test_control_authority)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp")
  add_executable(test_teleop_safety
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp"
    nav/safety/command.cpp
  )
  target_include_directories(test_teleop_safety PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_teleop_safety)
  add_test(NAME test_teleop_safety COMMAND test_teleop_safety)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp")
  add_executable(test_nav_endpoint_config
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp"
    nav/runtime/config/parse.cpp
    nav/runtime/config/build.cpp)
  target_include_directories(test_nav_endpoint_config PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    "${_EXPLORE_CPP_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_nav_endpoint_config PRIVATE
    lingtu_nav_far
    lingtu_nav_follower
    lingtu_nav_local_planner)
  lingtu_endpoint_enable_warnings(test_nav_endpoint_config)
  add_test(NAME test_nav_endpoint_config COMMAND test_nav_endpoint_config)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_debug_sampling.cpp")
  add_executable(test_nav_status_debug_sampling
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_status_debug_sampling.cpp")
  target_include_directories(test_nav_status_debug_sampling PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  lingtu_endpoint_enable_warnings(test_nav_status_debug_sampling)
  add_test(
    NAME test_nav_status_debug_sampling
    COMMAND test_nav_status_debug_sampling)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp")
  add_executable(test_status_snapshot_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp"
    nav/status/status_snapshot_file_writer.cpp)
  target_include_directories(test_status_snapshot_file_writer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_LINGTU_ROOT}/src")
  target_link_libraries(test_status_snapshot_file_writer PRIVATE Threads::Threads)
  lingtu_endpoint_enable_warnings(test_status_snapshot_file_writer)
  add_test(
    NAME test_status_snapshot_file_writer
    COMMAND test_status_snapshot_file_writer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_dds_drain_policy.cpp")
  add_executable(test_dds_drain_policy "${_NAV_ENDPOINT_TESTS_DIR}/test_dds_drain_policy.cpp")
  target_include_directories(test_dds_drain_policy PRIVATE
    ${_EP_DIRS}
    "${CMAKE_CURRENT_LIST_DIR}")
  lingtu_endpoint_enable_warnings(test_dds_drain_policy)
  add_test(NAME test_dds_drain_policy COMMAND test_dds_drain_policy)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp")
  add_executable(test_active_octomap_gate
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp"
    nav/input/active/octomap.cpp
    nav/input/active/declared.cpp)
  target_include_directories(test_active_octomap_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_OCTOPLANNER3D_RUNTIME_DIR}")
  target_link_libraries(test_active_octomap_gate PRIVATE
    lingtu_maps
    octoplanner3d_runtime
    octoplanner3d_global_planner)
  lingtu_endpoint_enable_warnings(test_active_octomap_gate)
  add_test(NAME test_active_octomap_gate COMMAND test_active_octomap_gate)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_occupancy_gate.cpp")
  add_executable(test_active_occupancy_gate
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_occupancy_gate.cpp"
    nav/input/active/occupancy.cpp
    nav/input/active/declared.cpp)
  target_include_directories(test_active_occupancy_gate PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_CORE_DIR}")
  target_link_libraries(test_active_occupancy_gate PRIVATE
    lingtu_maps
    lingtu_nav_far)
  lingtu_endpoint_enable_warnings(test_active_occupancy_gate)
  add_test(NAME test_active_occupancy_gate COMMAND test_active_occupancy_gate)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_saved_coverage_grid.cpp")
  add_executable(test_saved_coverage_grid
    "${_NAV_ENDPOINT_TESTS_DIR}/test_saved_coverage_grid.cpp"
    explore/saved_coverage_grid.cpp)
  target_include_directories(test_saved_coverage_grid PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_CORE_DIR}")
  target_link_libraries(test_saved_coverage_grid PRIVATE lingtu_nav_far)
  lingtu_endpoint_enable_warnings(test_saved_coverage_grid)
  add_test(NAME test_saved_coverage_grid COMMAND test_saved_coverage_grid)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  add_executable(test_pose_buffer "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  target_include_directories(test_pose_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_pose_buffer)
  add_test(NAME test_pose_buffer COMMAND test_pose_buffer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  add_executable(test_grid_inflation "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  target_include_directories(test_grid_inflation PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_grid_inflation)
  add_test(NAME test_grid_inflation COMMAND test_grid_inflation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  add_executable(test_point_cloud_layout "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  target_include_directories(test_point_cloud_layout PRIVATE "${CMAKE_CURRENT_LIST_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_point_cloud_layout)
  add_test(NAME test_point_cloud_layout COMMAND test_point_cloud_layout)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp")
  add_executable(test_transform_buffer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp"
    nav/dds/frame.cpp
    nav/input/frame.cpp)
  target_include_directories(test_transform_buffer PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_transform_buffer PRIVATE
    lingtu_dds_messages)
  lingtu_endpoint_enable_warnings(test_transform_buffer)
  add_test(NAME test_transform_buffer COMMAND test_transform_buffer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_traversability_geometry.cpp")
  add_executable(test_traversability_geometry
    "${_NAV_ENDPOINT_TESTS_DIR}/test_traversability_geometry.cpp"
    nav/dds/frame.cpp
    nav/input/frame.cpp)
  target_include_directories(test_traversability_geometry PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_traversability_geometry PRIVATE
    lingtu_dds_messages)
  lingtu_endpoint_enable_warnings(test_traversability_geometry)
  add_test(NAME test_traversability_geometry COMMAND test_traversability_geometry)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_observed_free_cache.cpp")
  add_executable(test_observed_free_cache "${_NAV_ENDPOINT_TESTS_DIR}/test_observed_free_cache.cpp")
  target_include_directories(test_observed_free_cache PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_observed_free_cache)
  add_test(NAME test_observed_free_cache COMMAND test_observed_free_cache)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_safety_grid_probe.cpp")
  add_executable(test_safety_grid_probe "${_NAV_ENDPOINT_TESTS_DIR}/test_safety_grid_probe.cpp")
  target_include_directories(test_safety_grid_probe PRIVATE
    "${_LINGTU_ROOT}/src"
    "${_LINGTU_ROOT}/src/maps/include"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_safety_grid_probe)
  add_test(NAME test_safety_grid_probe COMMAND test_safety_grid_probe)
endif()

find_package(GTest CONFIG QUIET)

if(TARGET GTest::gtest_main AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_assisted_teleop.cpp")
  add_executable(test_assisted_teleop
    "${_NAV_ENDPOINT_TESTS_DIR}/test_assisted_teleop.cpp"
    nav/control/authority.cpp
  )
  target_include_directories(test_assisted_teleop PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  target_compile_definitions(test_assisted_teleop PRIVATE
    LINGTU_TEST_PATH_LIBRARY="${_NAV_CPP_DIR}/planning/local/cmu/paths/go2")
  target_link_libraries(test_assisted_teleop PRIVATE
    lingtu_nav_navigation
    GTest::gtest_main)
  lingtu_endpoint_enable_warnings(test_assisted_teleop)
  add_test(NAME test_assisted_teleop COMMAND test_assisted_teleop)
endif()

endif()

add_executable(lingtu_traversability_dds
    nav/dds/frame.cpp
    nav/input/frame.cpp
    nav/input/gate.cpp
    traversability/main.cpp
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
    lingtu_dds_contracts
    Threads::Threads)

  if(OpenMP_CXX_FOUND)
    target_link_libraries(lingtu_traversability_dds PRIVATE OpenMP::OpenMP_CXX)
  endif()

  lingtu_endpoint_enable_warnings(lingtu_traversability_dds)
  lingtu_nav_enable_performance(lingtu_traversability_dds)
  set_target_properties(lingtu_traversability_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")


add_executable(lingtu_explore_dds
    nav/dds/frame.cpp
    nav/input/frame.cpp
    explore/main.cpp
    explore/control.cpp
    explore/run_event_outbox.cpp
    explore/run_lifecycle.cpp
    explore/directed_intent.cpp
    explore/goal_command_lane.cpp
    explore/goal_lifecycle.cpp
    explore/input_gate.cpp
    explore/saved_coverage_grid.cpp
    nav/input/active/occupancy.cpp
    nav/input/active/declared.cpp
    nav/status/status_snapshot_file_writer.cpp
    "${_EXPLORE_CPP_DIR}/frontier.cpp"
    "${_EXPLORE_CPP_DIR}/keypose_graph.cpp"
    "${_EXPLORE_CPP_DIR}/tare_policy.cpp")

  target_include_directories(lingtu_explore_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_CORE_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})

  target_link_libraries(lingtu_explore_dds PRIVATE
    lingtu_dds_messages
    lingtu_maps
    lingtu_nav_far
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_explore_dds)
  set_target_properties(lingtu_explore_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp")
  add_executable(test_explore_control
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp"
    explore/control.cpp)
  target_include_directories(test_explore_control PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_explore_control)
  lingtu_endpoint_enable_assertions(test_explore_control)
  add_test(NAME test_explore_control COMMAND test_explore_control)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_exploration_run_event_outbox.cpp")
  add_executable(test_exploration_run_event_outbox
    "${_NAV_ENDPOINT_TESTS_DIR}/test_exploration_run_event_outbox.cpp"
    explore/run_event_outbox.cpp
    explore/run_lifecycle.cpp)
  target_include_directories(test_exploration_run_event_outbox PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_exploration_run_event_outbox)
  lingtu_endpoint_enable_assertions(test_exploration_run_event_outbox)
  add_test(
    NAME test_exploration_run_event_outbox
    COMMAND test_exploration_run_event_outbox)
endif()

if(NOT TARGET test_exploration_snapshot_identity
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_exploration_snapshot_identity.cpp")
  add_executable(test_exploration_snapshot_identity
    "${_NAV_ENDPOINT_TESTS_DIR}/test_exploration_snapshot_identity.cpp")
  target_include_directories(test_exploration_snapshot_identity PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}")
  lingtu_endpoint_enable_warnings(test_exploration_snapshot_identity)
  add_test(
    NAME test_exploration_snapshot_identity
    COMMAND test_exploration_snapshot_identity)
endif()

if(NOT TARGET test_directed_exploration_intent
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp")
  add_executable(test_directed_exploration_intent
    "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp"
    explore/directed_intent.cpp)
  target_include_directories(test_directed_exploration_intent PRIVATE
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_directed_exploration_intent)
  lingtu_endpoint_enable_assertions(test_directed_exploration_intent)
  add_test(
    NAME test_directed_exploration_intent
    COMMAND test_directed_exploration_intent)
endif()

if(NOT TARGET test_explore_goal_lifecycle
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_lifecycle.cpp")
  add_executable(test_explore_goal_lifecycle
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_lifecycle.cpp"
    explore/goal_lifecycle.cpp)
  target_include_directories(test_explore_goal_lifecycle PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_explore_goal_lifecycle)
  lingtu_endpoint_enable_assertions(test_explore_goal_lifecycle)
  add_test(
    NAME test_explore_goal_lifecycle
    COMMAND test_explore_goal_lifecycle)
endif()

if(NOT TARGET test_explore_goal_command_lane
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_command_lane.cpp")
  add_executable(test_explore_goal_command_lane
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_goal_command_lane.cpp"
    explore/goal_command_lane.cpp)
  target_include_directories(test_explore_goal_command_lane PRIVATE
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_explore_goal_command_lane)
  lingtu_endpoint_enable_assertions(test_explore_goal_command_lane)
  add_test(
    NAME test_explore_goal_command_lane
    COMMAND test_explore_goal_command_lane)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_input.cpp")
  add_executable(test_explore_input
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_input.cpp"
    nav/dds/frame.cpp
    nav/input/frame.cpp
    explore/input_gate.cpp)
  target_include_directories(test_explore_input PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_KERNEL_DIR}/include"
    ${_EP_DIRS})
  target_link_libraries(test_explore_input PRIVATE
    lingtu_dds_messages)
  lingtu_endpoint_enable_warnings(test_explore_input)
  lingtu_endpoint_enable_assertions(test_explore_input)
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
  lingtu_endpoint_enable_warnings(test_tare_policy)
  lingtu_endpoint_enable_assertions(test_tare_policy)
  add_test(NAME test_tare_policy COMMAND test_tare_policy)
endif()
endif()
add_executable(lingtu_nav_control
    tools/navctl.cpp)

  target_include_directories(lingtu_nav_control PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

  target_link_libraries(lingtu_nav_control PRIVATE
    lingtu_nav_client
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_nav_control)
  set_target_properties(lingtu_nav_control PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    BUILD_RPATH "$ORIGIN")

add_executable(lingtu_motion_mock_dds
    tools/mock.cpp)

  target_include_directories(lingtu_motion_mock_dds PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})

  target_link_libraries(lingtu_motion_mock_dds PRIVATE
    lingtu_dds_messages
    CycloneDDS::ddsc
    lingtu_dds_contracts
    Threads::Threads)

  lingtu_endpoint_enable_warnings(lingtu_motion_mock_dds)
  set_target_properties(lingtu_motion_mock_dds PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

set(_LINGTU_NAV_ENDPOINT_RUNTIME_TARGETS)
foreach(_target IN ITEMS
    navd
    lingtu_traversability_dds
    lingtu_explore_dds
    lingtu_nav_control)
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

set(_LINGTU_CMU_PATH_SOURCE "${_NAV_CPP_DIR}/planning/local/cmu/paths")
set(_LINGTU_CMU_PATH_BUILD "${CMAKE_BINARY_DIR}/cmu_paths")
add_custom_target(lingtu_cmu_paths ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${_LINGTU_CMU_PATH_BUILD}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${_LINGTU_CMU_PATH_SOURCE}/go2"
    "${_LINGTU_CMU_PATH_BUILD}/go2"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${_LINGTU_CMU_PATH_SOURCE}/thunder"
    "${_LINGTU_CMU_PATH_BUILD}/thunder"
  COMMENT "Staging CMU path libraries for Go2 and Thunder")

install(DIRECTORY
  "${_LINGTU_CMU_PATH_SOURCE}/go2"
  "${_LINGTU_CMU_PATH_SOURCE}/thunder"
  DESTINATION cmu_paths
  COMPONENT cmu_paths)

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT TARGET test_inspection_task_event_outbox
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_task_event_outbox.cpp")
  add_executable(test_inspection_task_event_outbox
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_task_event_outbox.cpp")
  target_include_directories(test_inspection_task_event_outbox PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${_LINGTU_ROOT}/src"
    ${_EP_DIRS})
  target_link_libraries(test_inspection_task_event_outbox PRIVATE
    lingtu_inspection_core)
  lingtu_endpoint_enable_warnings(test_inspection_task_event_outbox)
  lingtu_endpoint_enable_assertions(test_inspection_task_event_outbox)
  add_test(
    NAME test_inspection_task_event_outbox
    COMMAND test_inspection_task_event_outbox)
endif()

if(WIN32)
  # Do not maintain a hand-written allowlist: every endpoint-owned executable
  # or shared library that directly links CycloneDDS must be independently
  # runnable after a targeted build. This also covers new DDS tests and tools.
  get_property(
    _LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS
    DIRECTORY
    PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_target IN LISTS _LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS)
    get_target_property(_target_type ${_target} TYPE)
    if(_target_type STREQUAL "EXECUTABLE"
        OR _target_type STREQUAL "SHARED_LIBRARY"
        OR _target_type STREQUAL "MODULE_LIBRARY")
      get_target_property(_target_link_libraries ${_target} LINK_LIBRARIES)
      if(_target_link_libraries)
        list(FIND _target_link_libraries "CycloneDDS::ddsc" _ddsc_link_index)
        if(NOT _ddsc_link_index EQUAL -1)
          lingtu_endpoint_bundle_cyclonedds(${_target})
        endif()
      endif()
    endif()
  endforeach()
  unset(_ddsc_link_index)
  unset(_target_link_libraries)
  unset(_target_type)
  unset(_LINGTU_NAV_ENDPOINT_DIRECTORY_TARGETS)
endif()

install(TARGETS
  lingtu_nav_client
  lingtu_inspection_evidence_bridge
  LIBRARY DESTINATION .
  RUNTIME DESTINATION .)
