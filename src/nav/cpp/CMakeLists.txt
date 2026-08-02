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

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_snapshot_identity.cpp")
  add_executable(test_exploration_snapshot_identity
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_snapshot_identity.cpp")
  target_include_directories(test_exploration_snapshot_identity PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint")
  target_compile_options(test_exploration_snapshot_identity PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_exploration_snapshot_identity
    COMMAND test_exploration_snapshot_identity)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_global_plan_task.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_global_plan_task
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp")
  target_include_directories(test_global_plan_task PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_global_plan_task PRIVATE Threads::Threads)
  target_compile_options(test_global_plan_task PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_global_plan_task COMMAND test_global_plan_task)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_control.cpp")
  add_executable(test_explore_control
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_control.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/explore/explore_control.cpp")
  target_include_directories(test_explore_control PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}")
  target_compile_options(test_explore_control PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_explore_control COMMAND test_explore_control)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_run_event_outbox.cpp")
  add_executable(test_exploration_run_event_outbox
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_run_event_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/explore/exploration_run_event_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/explore/exploration_run_lifecycle.cpp")
  target_include_directories(test_exploration_run_event_outbox PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}")
  target_compile_options(test_exploration_run_event_outbox PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_exploration_run_event_outbox
    COMMAND test_exploration_run_event_outbox)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS "${LINGTU_SRC_ROOT}/explore/cpp/test_tare_policy.cpp")
  set(_LINGTU_TARE_POLICY_DIR "${LINGTU_SRC_ROOT}/explore/cpp")
  add_executable(test_tare_policy
    "${_LINGTU_TARE_POLICY_DIR}/test_tare_policy.cpp"
    "${_LINGTU_TARE_POLICY_DIR}/frontier.cpp"
    "${_LINGTU_TARE_POLICY_DIR}/keypose_graph.cpp"
    "${_LINGTU_TARE_POLICY_DIR}/tare_policy.cpp")
  target_include_directories(test_tare_policy PRIVATE
    "${_LINGTU_TARE_POLICY_DIR}")
  target_compile_options(test_tare_policy PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -UNDEBUG>)
  add_test(NAME test_tare_policy COMMAND test_tare_policy)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_saved_coverage_grid.cpp")
  add_executable(test_saved_coverage_grid
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_saved_coverage_grid.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/explore/saved_coverage_grid.cpp")
  target_include_directories(test_saved_coverage_grid PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}/explore/cpp"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}")
  target_link_libraries(test_saved_coverage_grid PRIVATE lingtu_nav_far)
  target_compile_options(test_saved_coverage_grid PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -UNDEBUG>)
  add_test(NAME test_saved_coverage_grid COMMAND test_saved_coverage_grid)
endif()

add_library(lingtu_nav_velocity_smoother STATIC
  control/velocity_smoother.cpp)
add_library(LingTuNav::velocity_smoother ALIAS
  lingtu_nav_velocity_smoother)
target_compile_features(lingtu_nav_velocity_smoother PUBLIC cxx_std_17)
target_include_directories(lingtu_nav_velocity_smoother PUBLIC
  "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include>")
target_compile_options(lingtu_nav_velocity_smoother PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
set_target_properties(lingtu_nav_velocity_smoother PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
lingtu_nav_enable_performance(lingtu_nav_velocity_smoother)

if(LINGTU_NAV_CPP_BUILD_TESTS)
  add_executable(test_velocity_smoother
    "${LINGTU_NAV_TESTS_DIR}/test_velocity_smoother.cpp")
  target_link_libraries(test_velocity_smoother PRIVATE
    lingtu_nav_velocity_smoother)
  target_compile_options(test_velocity_smoother PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_velocity_smoother COMMAND test_velocity_smoother)
endif()

set(_LINGTU_ROLLING_SEGMENT_PLAN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan")
set(_LINGTU_EXPLORE_CPP_DIR
  "${LINGTU_SRC_ROOT}/explore/cpp")

if(EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/segment_executor.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_lifecycle.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/rolling_segment_effect_coordinator.cpp")
  add_library(lingtu_nav_rolling_segment STATIC
    "${_LINGTU_ROLLING_SEGMENT_PLAN_DIR}/segment_executor.cpp"
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_bounded_goal_replan_controller.cpp")
  add_executable(test_bounded_goal_replan_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp")
  target_include_directories(test_bounded_goal_replan_controller PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}")
  target_compile_options(test_bounded_goal_replan_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_bounded_goal_replan_controller
    COMMAND test_bounded_goal_replan_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_path_blockage_policy.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/active_path_blockage_policy.cpp")
  add_executable(test_active_path_blockage_policy
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_path_blockage_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/active_path_blockage_policy.cpp")
  target_include_directories(test_active_path_blockage_policy PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_compile_options(test_active_path_blockage_policy PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_active_path_blockage_policy
    COMMAND test_active_path_blockage_policy)
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_replan_runtime_coordinator.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_replan_runtime_coordinator
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_replan_runtime_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/command_ingress_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/goal_task_cancel_router.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_replan_runtime_coordinator.cpp")
  target_include_directories(test_goal_replan_runtime_coordinator PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_replan_runtime_coordinator PRIVATE Threads::Threads)
  target_compile_options(test_goal_replan_runtime_coordinator PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_replan_runtime_coordinator
    COMMAND test_goal_replan_runtime_coordinator)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_path_replan_cycle.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_active_path_replan_cycle
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_path_replan_cycle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/active_path_blockage_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/autonomy_tick_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_safety.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_replan_runtime_coordinator.cpp")
  target_include_directories(test_active_path_replan_cycle PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_active_path_replan_cycle PRIVATE Threads::Threads)
  target_compile_options(test_active_path_replan_cycle PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_active_path_replan_cycle
    COMMAND test_active_path_replan_cycle)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_ingress_policy.cpp")
  add_executable(test_goal_terminal_ingress_policy
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_ingress_policy.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_terminal_ingress_policy.cpp")
  target_include_directories(test_goal_terminal_ingress_policy PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint")
  target_compile_options(test_goal_terminal_ingress_policy PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_terminal_ingress_policy
    COMMAND test_goal_terminal_ingress_policy)
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
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_goal_status_outbox.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_goal_status_outbox.cpp")
  add_executable(test_navigation_goal_status_outbox
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_goal_status_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_goal_status_outbox.cpp")
  target_include_directories(test_navigation_goal_status_outbox PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_compile_options(test_navigation_goal_status_outbox PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_navigation_goal_status_outbox
    COMMAND test_navigation_goal_status_outbox)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_status_delivery.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_status_delivery.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_status_delivery
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_status_delivery.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_status_delivery.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_goal_status_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_replan_runtime_coordinator.cpp")
  target_include_directories(test_goal_terminal_status_delivery PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_terminal_status_delivery PRIVATE Threads::Threads)
  target_compile_options(test_goal_terminal_status_delivery PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_terminal_status_delivery
    COMMAND test_goal_terminal_status_delivery)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_transaction.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_transaction.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_transaction
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_transaction.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_status_delivery.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_transaction.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_goal_status_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_replan_runtime_coordinator.cpp")
  target_include_directories(test_goal_terminal_transaction PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_terminal_transaction PRIVATE Threads::Threads)
  target_compile_options(test_goal_terminal_transaction PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_terminal_transaction
    COMMAND test_goal_terminal_transaction)
endif()
if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_endpoint_acceptance.cpp"
    AND EXISTS
      "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_status_delivery.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_endpoint_acceptance
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_endpoint_acceptance.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/goal_terminal_status_delivery.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status/navigation_goal_status_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/motion_stop_coordinator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/global_plan_task.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/bounded_goal_replan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_plan_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/goal_replan_runtime_coordinator.cpp")
  target_include_directories(test_goal_terminal_endpoint_acceptance PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/status"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_goal_terminal_endpoint_acceptance PRIVATE Threads::Threads)
  target_compile_options(test_goal_terminal_endpoint_acceptance PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_goal_terminal_endpoint_acceptance
    COMMAND test_goal_terminal_endpoint_acceptance)
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_task_event_outbox.cpp")
  add_executable(test_inspection_task_event_outbox
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_task_event_outbox.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection/inspection_task_event_outbox.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_task_event_outbox PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/inspection")
  target_compile_options(test_inspection_task_event_outbox PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_inspection_task_event_outbox
    COMMAND test_inspection_task_event_outbox)
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_safety.cpp")
  add_executable(test_teleop_safety
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_teleop_safety.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion/teleop_safety.cpp")
  target_include_directories(test_teleop_safety PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/motion"
    "${LINGTU_NAV_PLAN_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_LOCAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_teleop_safety PRIVATE
    lingtu_nav_plan_loop)
  target_compile_options(test_teleop_safety PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_teleop_safety
    COMMAND test_teleop_safety)
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
    "${LINGTU_NAV_KERNEL_DIR}/include"
    "${_LINGTU_EXPLORE_CPP_DIR}")
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_declared_map_identity.cpp")
  add_executable(test_declared_map_identity
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_declared_map_identity.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan/active_map_declared_identity.cpp"
    "${LINGTU_SRC_ROOT}/maps/cpp/store.cpp"
    "${LINGTU_SRC_ROOT}/maps/cpp/hash.cpp"
    "${LINGTU_SRC_ROOT}/maps/cpp/lock.cpp"
    "${LINGTU_SRC_ROOT}/maps/cpp/version.cpp"
    "${LINGTU_SRC_ROOT}/maps/cpp/build/grid_artifacts.cpp")
  target_include_directories(test_declared_map_identity PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_SRC_ROOT}/maps/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint"
    "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/plan"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_GLOBAL_CPP_DIR}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_compile_options(test_declared_map_identity PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_declared_map_identity
    COMMAND test_declared_map_identity)
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
  lingtu_nav_velocity_smoother
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
