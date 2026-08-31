# Endpoint-focused tests. Production endpoint targets remain under ../../endpoint.

set(_ENDPOINT_DIR "${LINGTU_NAV_CPP_ROOT}/endpoint")
set(_LINGTU_ROOT "${LINGTU_NAV_CPP_ROOT}/../../..")
set(_NAV_COMMANDS_CPP_DIR "${LINGTU_NAV_CPP_ROOT}/client")
set(_NAV_TESTS_DIR "${LINGTU_NAV_CPP_ROOT}/tests")
set(_NAV_ENDPOINT_TESTS_DIR "${_NAV_TESTS_DIR}/endpoint")
set(_EXPLORE_CPP_DIR "${LINGTU_SRC_ROOT}/explore/cpp")
set(_LINGTU_EXPLORE_CPP_DIR "${_EXPLORE_CPP_DIR}")
set(_PERCEPTION_INSPECTION_DIR "${LINGTU_SRC_ROOT}/perception/inspection")
set(_NAV_CPP_DIR "${LINGTU_NAV_CPP_ROOT}")
set(_NAV_CORE_DIR "${LINGTU_NAV_CPP_ROOT}")
set(_NAV_KERNEL_DIR "${LINGTU_NAV_KERNEL_DIR}")
set(_OCTOPLANNER3D_RUNTIME_DIR
  "${LINGTU_NAV_CPP_ROOT}/planning/global/octoplanner")
set(_EP_DIRS
  "${_ENDPOINT_DIR}"
  "${_ENDPOINT_DIR}/nav"
  "${LINGTU_SRC_ROOT}")

if(LINGTU_NAV_CPP_BUILD_ENDPOINT)
  find_package(CycloneDDS REQUIRED)
  if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  endif()
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_exploration_snapshot_identity
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_snapshot_identity.cpp")
  add_executable(test_exploration_snapshot_identity
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_exploration_snapshot_identity.cpp")
  target_include_directories(test_exploration_snapshot_identity PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_exploration_snapshot_identity PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_exploration_snapshot_identity
    COMMAND test_exploration_snapshot_identity)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_directed_exploration_intent
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_directed_exploration_intent.cpp")
  add_executable(test_directed_exploration_intent
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_directed_exploration_intent.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/directed_intent.cpp")
  target_include_directories(test_directed_exploration_intent PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_SRC_ROOT}/explore/cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_directed_exploration_intent PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_directed_exploration_intent
    COMMAND test_directed_exploration_intent)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_explore_goal_lifecycle
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_goal_lifecycle.cpp")
  add_executable(test_explore_goal_lifecycle
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_goal_lifecycle.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/goal_lifecycle.cpp")
  target_include_directories(test_explore_goal_lifecycle PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_explore_goal_lifecycle PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_explore_goal_lifecycle
    COMMAND test_explore_goal_lifecycle)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND NOT TARGET test_explore_goal_command_lane
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_goal_command_lane.cpp")
  add_executable(test_explore_goal_command_lane
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_explore_goal_command_lane.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/goal_command_lane.cpp")
  target_include_directories(test_explore_goal_command_lane PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_explore_goal_command_lane PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_explore_goal_command_lane
    COMMAND test_explore_goal_command_lane)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_latest_slow_terrain_worker.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_latest_slow_terrain_worker
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_latest_slow_terrain_worker.cpp")
  target_include_directories(test_latest_slow_terrain_worker PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}/maps/include")
  target_link_libraries(test_latest_slow_terrain_worker PRIVATE Threads::Threads)
  target_compile_options(test_latest_slow_terrain_worker PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_latest_slow_terrain_worker
    COMMAND test_latest_slow_terrain_worker)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_global_plan_task.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_global_plan_task
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_global_plan_task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp")
  target_include_directories(test_global_plan_task PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/control.cpp")
  target_include_directories(test_explore_control PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/run_event_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/run_lifecycle.cpp")
  target_include_directories(test_exploration_run_event_outbox PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/explore/saved_coverage_grid.cpp")
  target_include_directories(test_saved_coverage_grid PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}/explore/cpp"
    "${LINGTU_NAV_CPP_ROOT}")
  target_link_libraries(test_saved_coverage_grid PRIVATE lingtu_nav_far)
  target_compile_options(test_saved_coverage_grid PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -UNDEBUG>)
  add_test(NAME test_saved_coverage_grid COMMAND test_saved_coverage_grid)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND TARGET lingtu_nav_rolling_segment
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_map_segment_executor.cpp")
  add_executable(test_rolling_map_segment_executor
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_rolling_map_segment_executor.cpp")
  target_include_directories(test_rolling_map_segment_executor PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp")
  target_include_directories(test_bounded_goal_replan_controller PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}")
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/blockage.cpp")
  add_executable(test_active_path_blockage_policy
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_active_path_blockage_policy.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/blockage.cpp")
  target_include_directories(test_active_path_blockage_policy PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp")
  target_include_directories(test_goal_plan_controller PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/command/cancel.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/command/ingress.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_replan_runtime_coordinator PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/blockage.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/autonomy.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/final.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/command.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_active_path_replan_cycle PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/command/terminal.cpp")
  target_include_directories(test_goal_terminal_ingress_policy PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp")
  target_include_directories(test_goal_plan_completion PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/command/ingress.cpp")
  target_include_directories(test_command_ingress_controller PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}")
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_stop_barrier.cpp")
  add_executable(test_motion_stop_barrier
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_motion_stop_barrier.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp")
  target_include_directories(test_motion_stop_barrier PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_motion_stop_barrier PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_motion_stop_barrier
    COMMAND test_motion_stop_barrier)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_health.cpp"
    AND EXISTS
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/control_loop_health.cpp")
  add_executable(test_control_loop_health
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_health.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/control_loop_health.cpp")
  target_include_directories(test_control_loop_health PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status")
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/guard.cpp")
  add_executable(test_control_loop_runtime_guard
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_control_loop_runtime_guard.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/guard.cpp")
  target_include_directories(test_control_loop_runtime_guard PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/nav_status_publisher.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/nav_status_writer.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/status_snapshot_file_writer.cpp")
  target_include_directories(test_nav_status_publisher PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_nav_status_publisher PRIVATE
    lingtu_nav_local_planner
    Threads::Threads)
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_state.cpp")
  add_executable(test_navigation_state
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_state.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_state.cpp")
  target_include_directories(test_navigation_state PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp")
  add_executable(test_navigation_goal_status_outbox
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_goal_status_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp")
  target_include_directories(test_navigation_goal_status_outbox PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_status_delivery
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_status_delivery.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_terminal_status_delivery PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_transaction.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_transaction
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_transaction.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_transaction.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_terminal_transaction PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_runtime_controller.cpp"
    AND EXISTS
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/navigation.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_navigation_runtime_controller
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_navigation_runtime_controller.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/navigation.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_transaction.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_navigation_runtime_controller PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_navigation_runtime_controller PRIVATE Threads::Threads)
  target_compile_options(test_navigation_runtime_controller PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(
    NAME test_navigation_runtime_controller
    COMMAND test_navigation_runtime_controller)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT LINGTU_NAV_CPP_BUILD_ENDPOINT
    AND EXISTS
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_endpoint_acceptance.cpp"
    AND EXISTS
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_goal_terminal_endpoint_acceptance
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_goal_terminal_endpoint_acceptance.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/goal_terminal_status_delivery.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/navigation_goal_status_outbox.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/stop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/task.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/retry.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/plan.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_terminal_endpoint_acceptance PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
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
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection/inspection_command_coordinator.cpp")
  add_executable(test_inspection_command_coordinator
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_command_coordinator.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection/inspection_command_coordinator.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_command_coordinator PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection")
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
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_task_event_outbox PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection/inspection_runtime_controller.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp")
  target_include_directories(test_inspection_runtime_controller PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/runtime/inspection")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/admission.cpp")
  target_include_directories(test_teleop_admission_controller PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/command.cpp")
  target_include_directories(test_teleop_safety PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_teleop_safety PRIVATE
    lingtu_nav_navigation)
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/teleop.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/final.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/command.cpp")
  target_include_directories(test_teleop_tick_controller PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include"
    "${_LINGTU_EXPLORE_CPP_DIR}")
  target_link_libraries(test_teleop_tick_controller PRIVATE
    lingtu_nav_navigation)
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/autonomy.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/final.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/safety/command.cpp")
  target_include_directories(test_autonomy_tick_controller PRIVATE
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_autonomy_tick_controller PRIVATE
    lingtu_nav_navigation)
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
      "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_status_file_writer.cpp")
  find_package(Threads REQUIRED)
  add_executable(test_inspection_status_file_writer
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_inspection_status_file_writer.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/inspection_status_file_writer.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/status_snapshot_file_writer.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/inspection.cpp"
    "${LINGTU_SRC_ROOT}/nav/inspection/store.cpp")
  target_include_directories(test_inspection_status_file_writer PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status")
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/authority.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/operator.cpp")
  target_include_directories(test_operator_motion_authority PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
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
    "${LINGTU_NAV_TESTS_DIR}/endpoint/test_operator_motion_output_evidence.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/control/operator.cpp")
  target_include_directories(test_operator_motion_output_evidence PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
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
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/input/obstacle.cpp")
  target_include_directories(test_motion_layer_portable PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")
  target_compile_options(test_motion_layer_portable PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_motion_layer_portable
    COMMAND test_motion_layer_portable)
endif()

if(LINGTU_NAV_CPP_BUILD_ENDPOINT)
if(LINGTU_NAV_CPP_BUILD_TESTS)
add_executable(test_nav_client
  "${_NAV_TESTS_DIR}/test_client.cpp"
  "${_ENDPOINT_DIR}/nav/command/ingress.cpp")
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

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp")
  add_executable(test_global_plan_task
    "${_NAV_ENDPOINT_TESTS_DIR}/test_global_plan_task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp")
  target_include_directories(test_global_plan_task PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/goal/retry.cpp")
  target_include_directories(test_bounded_goal_replan_controller PRIVATE
    "${_ENDPOINT_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_bounded_goal_replan_controller)
  add_test(NAME test_bounded_goal_replan_controller COMMAND test_bounded_goal_replan_controller)
endif()

if(NOT TARGET test_active_path_blockage_policy
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_blockage_policy.cpp"
    AND EXISTS "${_ENDPOINT_DIR}/nav/runtime/goal/blockage.cpp")
  add_executable(test_active_path_blockage_policy
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_path_blockage_policy.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/blockage.cpp")
  target_include_directories(test_active_path_blockage_policy PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp")
  target_include_directories(test_goal_plan_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/command/cancel.cpp"
    "${_ENDPOINT_DIR}/nav/command/ingress.cpp"
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/retry.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_replan_runtime_coordinator PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/goal/blockage.cpp"
    "${_ENDPOINT_DIR}/nav/control/autonomy.cpp"
    "${_ENDPOINT_DIR}/nav/control/final.cpp"
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp"
    "${_ENDPOINT_DIR}/nav/safety/command.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/retry.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_active_path_replan_cycle PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/status/goal_terminal_status_delivery.cpp"
    "${_ENDPOINT_DIR}/nav/status/goal_terminal_transaction.cpp"
    "${_ENDPOINT_DIR}/nav/status/navigation_goal_status_outbox.cpp"
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/retry.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_goal_terminal_transaction PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/navigation.cpp"
    "${_ENDPOINT_DIR}/nav/status/goal_terminal_status_delivery.cpp"
    "${_ENDPOINT_DIR}/nav/status/goal_terminal_transaction.cpp"
    "${_ENDPOINT_DIR}/nav/status/navigation_goal_status_outbox.cpp"
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/retry.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/runtime.cpp")
  target_include_directories(test_navigation_runtime_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/command/terminal.cpp")
  target_include_directories(test_goal_terminal_ingress_policy PRIVATE
    "${_ENDPOINT_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_goal_terminal_ingress_policy)
  add_test(NAME test_goal_terminal_ingress_policy
           COMMAND test_goal_terminal_ingress_policy)
endif()

if(NOT TARGET test_goal_plan_completion
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp")
  add_executable(test_goal_plan_completion
    "${_NAV_ENDPOINT_TESTS_DIR}/test_goal_plan_completion.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/task.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/goal/plan.cpp")
  target_include_directories(test_goal_plan_completion PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/input/obstacle.cpp")
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
    "${_ENDPOINT_DIR}/nav/input/obstacle.cpp")
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
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/planner.cpp"
    "${_ENDPOINT_DIR}/nav/input/obstacle.cpp"
    "${_ENDPOINT_DIR}/nav/safety/command.cpp"
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
    "${_ENDPOINT_DIR}/nav/dds/runtime.cpp"
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
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
    "${_ENDPOINT_DIR}/nav/dds/runtime.cpp"
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
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
    "${_ENDPOINT_DIR}/nav/dds/runtime.cpp"
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
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
  add_executable(test_input_gate "${_NAV_ENDPOINT_TESTS_DIR}/test_input_gate.cpp" "${_ENDPOINT_DIR}/nav/input/gate.cpp")
  target_include_directories(test_input_gate PRIVATE "${_ENDPOINT_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_input_gate)
  add_test(NAME test_input_gate COMMAND test_input_gate)
endif()
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_geofence.cpp")
  add_executable(test_geofence
    "${_NAV_ENDPOINT_TESTS_DIR}/test_geofence.cpp"
    "${_ENDPOINT_DIR}/nav/safety/geofence.cpp")
  target_include_directories(test_geofence PRIVATE ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_geofence)
  add_test(NAME test_geofence COMMAND test_geofence)
endif()
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_input_projector.cpp")
  add_executable(test_input_projector
    "${_NAV_ENDPOINT_TESTS_DIR}/test_input_projector.cpp"
    "${_ENDPOINT_DIR}/nav/input/pose.cpp"
    "${_ENDPOINT_DIR}/nav/input/map.cpp"
    "${_ENDPOINT_DIR}/nav/input/health.cpp"
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/planner.cpp"
    "${_ENDPOINT_DIR}/nav/input/gate.cpp"
    "${_ENDPOINT_DIR}/nav/input/obstacle.cpp")
  target_include_directories(test_input_projector PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/command/ingress.cpp")
  target_include_directories(test_command_ack_journal PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/command/ingress.cpp")
  target_include_directories(test_command_ingress_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp")
  target_include_directories(test_motion_stop_barrier PRIVATE
    "${_ENDPOINT_DIR}"
    ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_motion_stop_barrier)
  add_test(
    NAME test_motion_stop_barrier
    COMMAND test_motion_stop_barrier)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp")
  add_executable(test_operator_motion_dds
    "${_NAV_ENDPOINT_TESTS_DIR}/test_operator_motion_dds.cpp"
    "${_ENDPOINT_DIR}/nav/dds/runtime.cpp"
    "${_ENDPOINT_DIR}/nav/dds/codec.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/command/goal.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
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
    "${_ENDPOINT_DIR}/nav/control/operator.cpp"
    "${_ENDPOINT_DIR}/nav/control/authority.cpp")
  target_include_directories(test_operator_motion_authority PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/control/operator.cpp")
  target_include_directories(test_operator_motion_output_evidence PRIVATE
    "${_ENDPOINT_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_operator_motion_output_evidence)
  add_test(
    NAME test_operator_motion_output_evidence
    COMMAND test_operator_motion_output_evidence)
endif()
if(NOT TARGET test_control_loop_health
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    AND EXISTS "${_ENDPOINT_DIR}/nav/status/control_loop_health.cpp")
  add_executable(test_control_loop_health
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_health.cpp"
    "${_ENDPOINT_DIR}/nav/status/control_loop_health.cpp")
  target_include_directories(test_control_loop_health PRIVATE
    "${_ENDPOINT_DIR}"
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
      "${_ENDPOINT_DIR}/nav/control/guard.cpp")
  add_executable(test_control_loop_runtime_guard
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_loop_runtime_guard.cpp"
    "${_ENDPOINT_DIR}/nav/control/guard.cpp")
  target_include_directories(test_control_loop_runtime_guard PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/status/nav_status_publisher.cpp"
    "${_ENDPOINT_DIR}/nav/status/nav_status_writer.cpp"
    "${_ENDPOINT_DIR}/nav/status/status_snapshot_file_writer.cpp")
  target_include_directories(test_nav_status_publisher PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/status/navigation_state.cpp")
  target_include_directories(test_navigation_state PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/inspection/inspection_command_coordinator.cpp")
  target_include_directories(test_inspection_command_coordinator PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/runtime/inspection/inspection_runtime_controller.cpp")
  target_include_directories(test_inspection_runtime_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/control/admission.cpp")
  target_include_directories(test_teleop_admission_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/control/teleop.cpp"
    "${_ENDPOINT_DIR}/nav/control/final.cpp"
    "${_ENDPOINT_DIR}/nav/safety/command.cpp")
  target_include_directories(test_teleop_tick_controller PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/control/autonomy.cpp"
    "${_ENDPOINT_DIR}/nav/control/final.cpp"
    "${_ENDPOINT_DIR}/nav/safety/command.cpp")
  target_include_directories(test_autonomy_tick_controller PRIVATE
    "${_ENDPOINT_DIR}"
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

if(NOT TARGET test_inspection_status_file_writer
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp")
  add_executable(test_inspection_status_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_status_file_writer.cpp"
    "${_ENDPOINT_DIR}/nav/status/inspection_status_file_writer.cpp"
    "${_ENDPOINT_DIR}/nav/status/status_snapshot_file_writer.cpp")
  target_include_directories(test_inspection_status_file_writer PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp")
  target_include_directories(test_stop_confirmation PRIVATE
    ${_EP_DIRS}
    "${_ENDPOINT_DIR}")
  lingtu_endpoint_enable_warnings(test_stop_confirmation)
  add_test(NAME test_stop_confirmation COMMAND test_stop_confirmation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp")
  add_executable(test_control_authority
    "${_NAV_ENDPOINT_TESTS_DIR}/test_control_authority.cpp"
    "${_ENDPOINT_DIR}/nav/control/authority.cpp"
    "${_ENDPOINT_DIR}/nav/safety/stop.cpp")
  target_include_directories(test_control_authority PRIVATE
    "${_ENDPOINT_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_control_authority)
  add_test(NAME test_control_authority COMMAND test_control_authority)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp")
  add_executable(test_teleop_safety
    "${_NAV_ENDPOINT_TESTS_DIR}/test_teleop_safety.cpp"
    "${_ENDPOINT_DIR}/nav/safety/command.cpp"
  )
  target_include_directories(test_teleop_safety PRIVATE
    "${_ENDPOINT_DIR}"
    "${_NAV_CORE_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_teleop_safety)
  add_test(NAME test_teleop_safety COMMAND test_teleop_safety)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp")
  add_executable(test_nav_endpoint_config
    "${_NAV_ENDPOINT_TESTS_DIR}/test_nav_endpoint_config.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/config/parse.cpp"
    "${_ENDPOINT_DIR}/nav/runtime/config/build.cpp")
  target_include_directories(test_nav_endpoint_config PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}")
  lingtu_endpoint_enable_warnings(test_nav_status_debug_sampling)
  add_test(
    NAME test_nav_status_debug_sampling
    COMMAND test_nav_status_debug_sampling)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp")
  add_executable(test_status_snapshot_file_writer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_status_snapshot_file_writer.cpp"
    "${_ENDPOINT_DIR}/nav/status/status_snapshot_file_writer.cpp")
  target_include_directories(test_status_snapshot_file_writer PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}")
  lingtu_endpoint_enable_warnings(test_dds_drain_policy)
  add_test(NAME test_dds_drain_policy COMMAND test_dds_drain_policy)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp")
  add_executable(test_active_octomap_gate
    "${_NAV_ENDPOINT_TESTS_DIR}/test_active_octomap_gate.cpp"
    "${_ENDPOINT_DIR}/nav/input/active/octomap.cpp"
    "${_ENDPOINT_DIR}/nav/input/active/declared.cpp")
  target_include_directories(test_active_octomap_gate PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/input/active/occupancy.cpp"
    "${_ENDPOINT_DIR}/nav/input/active/declared.cpp")
  target_include_directories(test_active_occupancy_gate PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/explore/saved_coverage_grid.cpp")
  target_include_directories(test_saved_coverage_grid PRIVATE
    "${_ENDPOINT_DIR}"
    "${_EXPLORE_CPP_DIR}"
    "${_NAV_CORE_DIR}")
  target_link_libraries(test_saved_coverage_grid PRIVATE lingtu_nav_far)
  lingtu_endpoint_enable_warnings(test_saved_coverage_grid)
  add_test(NAME test_saved_coverage_grid COMMAND test_saved_coverage_grid)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  add_executable(test_pose_buffer "${_NAV_ENDPOINT_TESTS_DIR}/test_pose_buffer.cpp")
  target_include_directories(test_pose_buffer PRIVATE
    "${_ENDPOINT_DIR}"
    ${_EP_DIRS}
    "${_NAV_KERNEL_DIR}/include")
  lingtu_endpoint_enable_warnings(test_pose_buffer)
  add_test(NAME test_pose_buffer COMMAND test_pose_buffer)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  add_executable(test_grid_inflation "${_NAV_ENDPOINT_TESTS_DIR}/test_grid_inflation.cpp")
  target_include_directories(test_grid_inflation PRIVATE "${_ENDPOINT_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_grid_inflation)
  add_test(NAME test_grid_inflation COMMAND test_grid_inflation)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  add_executable(test_point_cloud_layout "${_NAV_ENDPOINT_TESTS_DIR}/test_point_cloud_layout.cpp")
  target_include_directories(test_point_cloud_layout PRIVATE "${_ENDPOINT_DIR}" ${_EP_DIRS})
  lingtu_endpoint_enable_warnings(test_point_cloud_layout)
  add_test(NAME test_point_cloud_layout COMMAND test_point_cloud_layout)
endif()

if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp")
  add_executable(test_transform_buffer
    "${_NAV_ENDPOINT_TESTS_DIR}/test_transform_buffer.cpp"
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
  target_include_directories(test_transform_buffer PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp")
  target_include_directories(test_traversability_geometry PRIVATE
    "${_ENDPOINT_DIR}"
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
    "${_ENDPOINT_DIR}/nav/control/authority.cpp"
  )
  target_include_directories(test_assisted_teleop PRIVATE
    "${_ENDPOINT_DIR}"
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

if(LINGTU_NAV_CPP_BUILD_TESTS)
if(EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp")
  add_executable(test_explore_control
    "${_NAV_ENDPOINT_TESTS_DIR}/test_explore_control.cpp"
    "${_ENDPOINT_DIR}/explore/control.cpp")
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
    "${_ENDPOINT_DIR}/explore/run_event_outbox.cpp"
    "${_ENDPOINT_DIR}/explore/run_lifecycle.cpp")
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
    "${_ENDPOINT_DIR}")
  lingtu_endpoint_enable_warnings(test_exploration_snapshot_identity)
  add_test(
    NAME test_exploration_snapshot_identity
    COMMAND test_exploration_snapshot_identity)
endif()

if(NOT TARGET test_directed_exploration_intent
    AND EXISTS "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp")
  add_executable(test_directed_exploration_intent
    "${_NAV_ENDPOINT_TESTS_DIR}/test_directed_exploration_intent.cpp"
    "${_ENDPOINT_DIR}/explore/directed_intent.cpp")
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
    "${_ENDPOINT_DIR}/explore/goal_lifecycle.cpp")
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
    "${_ENDPOINT_DIR}/explore/goal_command_lane.cpp")
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
    "${_ENDPOINT_DIR}/nav/dds/frame.cpp"
    "${_ENDPOINT_DIR}/nav/input/frame.cpp"
    "${_ENDPOINT_DIR}/explore/input_gate.cpp")
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

if(LINGTU_NAV_CPP_BUILD_TESTS
    AND NOT TARGET test_inspection_task_event_outbox
    AND EXISTS
      "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_task_event_outbox.cpp")
  add_executable(test_inspection_task_event_outbox
    "${_NAV_ENDPOINT_TESTS_DIR}/test_inspection_task_event_outbox.cpp")
  target_include_directories(test_inspection_task_event_outbox PRIVATE
    "${_ENDPOINT_DIR}"
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
endif()
