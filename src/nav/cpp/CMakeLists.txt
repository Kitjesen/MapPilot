cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(cmake/NavCoreTargets.cmake)

option(LINGTU_NAV_CPP_BUILD_TESTS
  "Build focused portable navigation C++ tests" ON)
option(LINGTU_NAV_CPP_BUILD_ENDPOINT
  "Build the Linux CycloneDDS navigation endpoint" OFF)
option(LINGTU_NAV_CPP_BUILD_PYTHON
  "Build the nanobind navigation extension" OFF)
option(LINGTU_NAV_CPP_BUILD_BENCHMARKS
  "Build optional navigation C++ micro-benchmarks" OFF)

set(_LINGTU_ROLLING_SEGMENT_PLAN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan")
set(_LINGTU_EXPLORE_CPP_DIR
  "${LINGTU_SRC_ROOT}/explore/cpp")

if(EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_map_segment_executor.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_lifecycle.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_effect_coordinator.cpp")
  add_library(lingtu_nav_rolling_segment STATIC
    "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_map_segment_executor.cpp"
    "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_lifecycle.cpp"
    "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_effect_coordinator.cpp")
  add_library(LingTuNav::rolling_segment ALIAS
    lingtu_nav_rolling_segment)
  target_compile_features(lingtu_nav_rolling_segment PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_rolling_segment PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/endpoint>"
    "$<BUILD_INTERFACE:${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}>"
    "$<BUILD_INTERFACE:${_LINGTU_EXPLORE_CPP_DIR}>")
  target_compile_options(lingtu_nav_rolling_segment PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  set_target_properties(lingtu_nav_rolling_segment PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  lingtu_nav_enable_performance(lingtu_nav_rolling_segment)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_map_segment_executor.cpp")
  add_executable(test_rolling_map_segment_executor
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_map_segment_executor.cpp")
  target_include_directories(test_rolling_map_segment_executor PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint")
  target_link_libraries(test_rolling_map_segment_executor PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_map_segment_executor PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_rolling_map_segment_executor
    COMMAND test_rolling_map_segment_executor)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_segment_lifecycle.cpp")
  add_executable(test_rolling_segment_lifecycle
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_segment_lifecycle.cpp")
  target_include_directories(test_rolling_segment_lifecycle PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint")
  target_link_libraries(test_rolling_segment_lifecycle PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_segment_lifecycle PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_rolling_segment_lifecycle
    COMMAND test_rolling_segment_lifecycle)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_segment_effect_coordinator.cpp")
  add_executable(test_rolling_segment_effect_coordinator
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_segment_effect_coordinator.cpp")
  target_include_directories(test_rolling_segment_effect_coordinator PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint")
  target_link_libraries(test_rolling_segment_effect_coordinator PRIVATE
    lingtu_nav_rolling_segment)
  target_compile_options(test_rolling_segment_effect_coordinator PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_rolling_segment_effect_coordinator
    COMMAND test_rolling_segment_effect_coordinator)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_plan_controller.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_plan_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp")
  target_include_directories(test_goal_plan_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_controller PRIVATE Threads::Threads)
  target_compile_options(test_goal_plan_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_plan_controller
    COMMAND test_goal_plan_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_plan_completion.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_plan_completion
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_plan_completion.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp")
  target_include_directories(test_goal_plan_completion PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_plan_completion PRIVATE Threads::Threads)
  target_compile_options(test_goal_plan_completion PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_plan_completion
    COMMAND test_goal_plan_completion)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_command_ingress_controller.cpp")
  add_executable(test_command_ingress_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_command_ingress_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/command_ingress_controller.cpp")
  target_include_directories(test_command_ingress_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion")
  target_compile_options(test_command_ingress_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_command_ingress_controller
    COMMAND test_command_ingress_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_stop_coordinator.cpp")
  add_executable(test_motion_stop_coordinator
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp")
  target_include_directories(test_motion_stop_coordinator PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion")
  target_compile_options(test_motion_stop_coordinator PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_motion_stop_coordinator
    COMMAND test_motion_stop_coordinator)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_health.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/control_loop_health.cpp")
  add_executable(test_control_loop_health
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_health.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/control_loop_health.cpp")
  target_include_directories(test_control_loop_health PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status")
  target_compile_options(test_control_loop_health PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_control_loop_health
    COMMAND test_control_loop_health)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_runtime_guard.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/control_loop_runtime_guard.cpp")
  add_executable(test_control_loop_runtime_guard
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_runtime_guard.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/control_loop_runtime_guard.cpp")
  target_include_directories(test_control_loop_runtime_guard PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status")
  target_compile_options(test_control_loop_runtime_guard PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_control_loop_runtime_guard
    COMMAND test_control_loop_runtime_guard)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_nav_status_publisher.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_nav_status_publisher
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_nav_status_publisher.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/nav_status_publisher.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/nav_status_writer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/status_snapshot_file_writer.cpp")
  target_include_directories(test_nav_status_publisher PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_nav_status_publisher PRIVATE Threads::Threads)
  target_compile_options(test_nav_status_publisher PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_nav_status_publisher
    COMMAND test_nav_status_publisher)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_state.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_state.cpp")
  add_executable(test_navigation_state
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_state.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_state.cpp")
  target_include_directories(test_navigation_state PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_compile_options(test_navigation_state PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_navigation_state
    COMMAND test_navigation_state)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_command_coordinator.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection/inspection_command_coordinator.cpp")
  add_executable(test_inspection_command_coordinator
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_command_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection/inspection_command_coordinator.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_command_coordinator PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection")
  target_compile_options(test_inspection_command_coordinator PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_inspection_command_coordinator
    COMMAND test_inspection_command_coordinator)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_runtime_controller.cpp")
  add_executable(test_inspection_runtime_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_runtime_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection/inspection_runtime_controller.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_runtime_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection")
  target_compile_options(test_inspection_runtime_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_inspection_runtime_controller
    COMMAND test_inspection_runtime_controller)
endif()



if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_admission_controller.cpp")
  add_executable(test_teleop_admission_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_admission_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_admission_controller.cpp")
  target_include_directories(test_teleop_admission_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_compile_options(test_teleop_admission_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_teleop_admission_controller
    COMMAND test_teleop_admission_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_tick_controller.cpp")
  add_executable(test_teleop_tick_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_tick_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_tick_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_safety.cpp")
  target_include_directories(test_teleop_tick_controller PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_teleop_tick_controller PRIVATE
    lingtu_nav_plan_loop)
  target_compile_options(test_teleop_tick_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_teleop_tick_controller
    COMMAND test_teleop_tick_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_autonomy_tick_controller.cpp")
  add_executable(test_autonomy_tick_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_autonomy_tick_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/autonomy_tick_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_safety.cpp")
  target_include_directories(test_autonomy_tick_controller PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_autonomy_tick_controller PRIVATE
    lingtu_nav_plan_loop)
  target_compile_options(test_autonomy_tick_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_autonomy_tick_controller
    COMMAND test_autonomy_tick_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_inspection_map_cache.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_active_inspection_map_cache
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_inspection_map_cache.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/active_inspection_map_cache.cpp")
  target_include_directories(test_active_inspection_map_cache PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status")
  target_link_libraries(test_active_inspection_map_cache PRIVATE
    Threads::Threads)
  target_compile_options(test_active_inspection_map_cache PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_active_inspection_map_cache
    COMMAND test_active_inspection_map_cache)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_status_file_writer.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_inspection_status_file_writer
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_status_file_writer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/inspection_status_file_writer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/status_snapshot_file_writer.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/store.cpp")
  target_include_directories(test_inspection_status_file_writer PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status")
  target_link_libraries(test_inspection_status_file_writer PRIVATE
    Threads::Threads)
  target_compile_options(test_inspection_status_file_writer PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_inspection_status_file_writer
    COMMAND test_inspection_status_file_writer)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_operator_motion_authority
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_operator_motion_authority.cpp")
  add_executable(test_operator_motion_authority
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_operator_motion_authority.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/operator_motion_authority.cpp")
  target_include_directories(test_operator_motion_authority PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${LINGTU_NAV_INCLUDE_DIR}")
  target_compile_options(test_operator_motion_authority PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_operator_motion_authority
    COMMAND test_operator_motion_authority)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_operator_motion_output_evidence
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_operator_motion_output_evidence.cpp")
  add_executable(test_operator_motion_output_evidence
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_operator_motion_output_evidence.cpp")
  target_include_directories(test_operator_motion_output_evidence PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${LINGTU_NAV_INCLUDE_DIR}")
  target_compile_options(test_operator_motion_output_evidence PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_operator_motion_output_evidence
    COMMAND test_operator_motion_output_evidence)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_layer.cpp")
  add_executable(test_motion_layer_portable
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_layer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_layer.cpp")
  target_include_directories(test_motion_layer_portable PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion")
  target_compile_options(test_motion_layer_portable PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_motion_layer_portable
    COMMAND test_motion_layer_portable)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT)
  find_package(GTest CONFIG QUIET)
  if(TARGET GTest::gtest_main)
    include(GoogleTest)

    function(lingtu_add_nav_gtest target source)
      add_executable(${target} "${LINGTU_NAV_TESTS_DIR}/${source}")
      target_include_directories(${target} PRIVATE
        "${LINGTU_NAV_INCLUDE_DIR}"
        "${LINGTU_NAV_LOCAL_CPP_DIR}"
        "${LINGTU_NAV_PLAN_CPP_DIR}")
      target_link_libraries(${target} PRIVATE
        lingtu_nav_path_follower
        lingtu_nav_local_planner
        GTest::gtest_main)
      gtest_discover_tests(${target})
    endfunction()

    foreach(test_name IN ITEMS
        test_path_follower_core
        test_pct_adapter_core
        test_param_sensitivity
        test_path_edge_cases
        test_benchmark
        test_validation
        test_terrain_core
        test_dynamic_clear_core
        test_far_c_api
        test_far_planner
        test_local_planner_core)
      lingtu_add_nav_gtest(${test_name} "${test_name}.cpp")
    endforeach()

    target_link_libraries(test_far_planner PRIVATE lingtu_nav_far)
    target_link_libraries(test_far_c_api PRIVATE lingtu_nav_far_c_api)

    lingtu_add_nav_gtest(test_nav_loop test_nav_loop.cpp)
    target_compile_definitions(test_nav_loop PRIVATE
      LINGTU_TEST_PATH_LIBRARY="${LINGTU_NAV_ROOT}/local/paths")
    target_link_libraries(test_nav_loop PRIVATE lingtu_nav_plan_loop)
  else()
    message(STATUS
      "GTest was not found; portable nav targets will build without tests")
  endif()
endif()

if(LINGTU_NAV_CPP_BUILD_BENCHMARKS
    AND EXISTS "${LINGTU_NAV_TESTS_DIR}/benchmark_local_planner_core.cpp")
  add_executable(benchmark_local_planner_core
    "${LINGTU_NAV_TESTS_DIR}/benchmark_local_planner_core.cpp")
  target_link_libraries(benchmark_local_planner_core PRIVATE
    lingtu_nav_local_planner)
  lingtu_nav_enable_performance(benchmark_local_planner_core)
endif()

if(LINGTU_NAV_CPP_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c
      "import nanobind; print(nanobind.cmake_dir())"
    OUTPUT_VARIABLE NANOBIND_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE NANOBIND_RESULT)
  if(NANOBIND_RESULT EQUAL 0)
    list(APPEND CMAKE_PREFIX_PATH "${NANOBIND_CMAKE_DIR}")
    find_package(nanobind CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(nanobind
      GIT_REPOSITORY https://github.com/wjakob/nanobind.git
      GIT_TAG v2.12.0)
    FetchContent_MakeAvailable(nanobind)
  endif()

  set(LINGTU_NAV_BINDING_SOURCES
    bindings/bindings.cpp
    bindings/bind_types.cpp
    bindings/bind_map_layers.cpp
    bindings/bind_path_follower.cpp
    bindings/bind_waypoint_helpers.cpp
    bindings/bind_local_planner.cpp
    bindings/bind_terrain.cpp)
  nanobind_add_module(lingtu_nav_kernel ${LINGTU_NAV_BINDING_SOURCES})
  target_include_directories(lingtu_nav_kernel PRIVATE
    "${LINGTU_NAV_INCLUDE_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_SRC_ROOT}/maps/include")
  target_link_libraries(lingtu_nav_kernel PRIVATE
    lingtu_nav_path_follower
    lingtu_nav_local_planner)
  lingtu_nav_enable_performance(lingtu_nav_kernel)
endif()

if(LINGTU_NAV_CPP_BUILD_ENDPOINT)
  if(NOT UNIX)
    message(FATAL_ERROR "The native navigation endpoint requires Linux")
  endif()
  add_subdirectory(endpoint)
endif()

install(TARGETS
  lingtu_nav_far
  lingtu_nav_far_c_api
  lingtu_nav_path_follower
  lingtu_nav_plan_loop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES
  planning/local/local_planner.hpp
  planning/local/local_planner_scoring.hpp
  DESTINATION include/nav/local)
install(FILES
  planning/global/global_planner_contract.hpp
  DESTINATION include/nav/global)
install(FILES
  planning/global/far/far_planner.hpp
  planning/global/far/far_c_api.h
  DESTINATION include/nav/global/far)
