get_filename_component(LINGTU_REPO_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(_EXPLORE_SRC "${LINGTU_REPO_ROOT}/src/explore/cpp")
set(_EXPLORE_ENDPOINT "${_EXPLORE_SRC}/endpoint")
set(_NAV_SRC "${LINGTU_REPO_ROOT}/src/nav/cpp")

function(lingtu_add_explore_test target)
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /UNDEBUG>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -UNDEBUG>)
  add_test(NAME ${target} COMMAND ${target})
endfunction()

if(TARGET lingtu_explore_core)
  add_executable(test_tare_policy test_tare_policy.cpp)
  target_link_libraries(test_tare_policy PRIVATE lingtu_explore_core)
elseif(DEFINED LINGTU_NAV_CPP_ROOT)
  add_executable(test_tare_policy
    test_tare_policy.cpp
    "${_EXPLORE_SRC}/frontier.cpp"
    "${_EXPLORE_SRC}/keypose_graph.cpp"
    "${_EXPLORE_SRC}/tare_policy.cpp")
  target_include_directories(test_tare_policy PRIVATE "${_EXPLORE_SRC}")
endif()
if(TARGET test_tare_policy)
  lingtu_add_explore_test(test_tare_policy)
endif()

if(NOT DEFINED LINGTU_NAV_CPP_ROOT OR NOT LINGTU_NAV_CPP_BUILD_TESTS)
  return()
endif()

set(_EXPLORE_INCLUDES
  "${LINGTU_REPO_ROOT}/src"
  "${_EXPLORE_SRC}"
  "${_NAV_SRC}"
  "${_NAV_SRC}/endpoint"
  "${_NAV_SRC}/endpoint/nav")

add_executable(test_exploration_snapshot_identity
  test_exploration_snapshot_identity.cpp)
target_include_directories(test_exploration_snapshot_identity PRIVATE
  "${_NAV_SRC}/endpoint")
lingtu_add_explore_test(test_exploration_snapshot_identity)

add_executable(test_directed_exploration_intent
  test_directed_exploration_intent.cpp
  "${_EXPLORE_ENDPOINT}/directed_intent.cpp")
target_include_directories(test_directed_exploration_intent PRIVATE
  ${_EXPLORE_INCLUDES})
lingtu_add_explore_test(test_directed_exploration_intent)

add_executable(test_explore_goal_lifecycle
  test_explore_goal_lifecycle.cpp
  "${_EXPLORE_ENDPOINT}/goal_lifecycle.cpp")
target_include_directories(test_explore_goal_lifecycle PRIVATE
  ${_EXPLORE_INCLUDES})
lingtu_add_explore_test(test_explore_goal_lifecycle)

add_executable(test_explore_goal_command_lane
  test_explore_goal_command_lane.cpp
  "${_EXPLORE_ENDPOINT}/goal_command_lane.cpp")
target_include_directories(test_explore_goal_command_lane PRIVATE
  ${_EXPLORE_INCLUDES})
lingtu_add_explore_test(test_explore_goal_command_lane)

add_executable(test_explore_control
  test_explore_control.cpp
  "${_EXPLORE_ENDPOINT}/control.cpp")
target_include_directories(test_explore_control PRIVATE ${_EXPLORE_INCLUDES})
lingtu_add_explore_test(test_explore_control)

add_executable(test_exploration_run_event_outbox
  test_exploration_run_event_outbox.cpp
  "${_EXPLORE_ENDPOINT}/run_event_outbox.cpp"
  "${_EXPLORE_ENDPOINT}/run_lifecycle.cpp")
target_include_directories(test_exploration_run_event_outbox PRIVATE
  ${_EXPLORE_INCLUDES})
lingtu_add_explore_test(test_exploration_run_event_outbox)

add_executable(test_saved_coverage_grid
  test_saved_coverage_grid.cpp
  "${_EXPLORE_ENDPOINT}/saved_coverage_grid.cpp")
target_include_directories(test_saved_coverage_grid PRIVATE
  ${_EXPLORE_INCLUDES})
target_link_libraries(test_saved_coverage_grid PRIVATE lingtu_nav_far)
lingtu_add_explore_test(test_saved_coverage_grid)

if(LINGTU_NAV_CPP_BUILD_ENDPOINT)
  add_executable(test_explore_input
    test_explore_input.cpp
    "${_NAV_SRC}/endpoint/nav/dds/frame.cpp"
    "${_NAV_SRC}/endpoint/nav/input/frame.cpp"
    "${_EXPLORE_ENDPOINT}/input_gate.cpp")
  target_include_directories(test_explore_input PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    ${_EXPLORE_INCLUDES}
    "${LINGTU_NAV_KERNEL_DIR}/include")
  target_link_libraries(test_explore_input PRIVATE lingtu_dds_messages)
  lingtu_add_explore_test(test_explore_input)
endif()
