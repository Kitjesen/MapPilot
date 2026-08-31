# Test and benchmark targets are owned by this directory.
if(LINGTU_NAV_CPP_BUILD_TESTS)
  add_executable(test_platform_runtime
    "${LINGTU_NAV_CPP_ROOT}/tests/test_platform_runtime.cpp")
  target_include_directories(test_platform_runtime PRIVATE
    "${LINGTU_NAV_CPP_ROOT}")
  target_compile_options(test_platform_runtime PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  add_test(NAME test_platform_runtime COMMAND test_platform_runtime)
endif()

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

if(LINGTU_NAV_CPP_BUILD_TESTS)
  find_package(GTest CONFIG QUIET)
  if(TARGET GTest::gtest_main)
    include(GoogleTest)

    function(lingtu_add_nav_gtest target source)
      add_executable(${target} "${LINGTU_NAV_TESTS_DIR}/${source}")
      target_include_directories(${target} PRIVATE
        "${LINGTU_NAV_INCLUDE_DIR}"
        "${LINGTU_NAV_CPP_ROOT}")
      target_link_libraries(${target} PRIVATE
        lingtu_nav_follower
        lingtu_nav_local_planner
        GTest::gtest_main)
      gtest_discover_tests(${target})
    endfunction()

    foreach(test_name IN ITEMS
        test_path_follower_core
        test_waypoint_helpers_core
        test_param_sensitivity
        test_path_edge_cases
        test_benchmark
        test_validation
        test_terrain_core
        test_dynamic_clear_core
        test_far_c_api
        test_far_planner
        test_scan_planner
        test_local_plan_task
        test_recovery_rotation
        test_recovery_sequence
        test_local_planner_core)
      lingtu_add_nav_gtest(${test_name} "${test_name}.cpp")
    endforeach()

    target_link_libraries(test_far_planner PRIVATE lingtu_nav_far)
    target_link_libraries(test_far_c_api PRIVATE lingtu_nav_far_c_api)
    set_target_properties(test_far_c_api PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

    lingtu_add_nav_gtest(test_executor test_executor.cpp)
    target_compile_definitions(test_executor PRIVATE
      LINGTU_TEST_PATH_LIBRARY="${LINGTU_NAV_ROOT}/cpp/planning/local/cmu/paths/go2")
    target_link_libraries(test_executor PRIVATE lingtu_nav_navigation)
    lingtu_add_nav_gtest(test_cmu_teleop test_cmu_teleop.cpp)
    target_compile_definitions(test_cmu_teleop PRIVATE
      LINGTU_TEST_GO2_PATH_LIBRARY="${LINGTU_NAV_ROOT}/cpp/planning/local/cmu/paths/go2"
      LINGTU_TEST_THUNDER_PATH_LIBRARY="${LINGTU_NAV_ROOT}/cpp/planning/local/cmu/paths/thunder")
    target_link_libraries(test_cmu_teleop PRIVATE lingtu_nav_navigation)
    target_link_libraries(test_recovery_sequence PRIVATE lingtu_nav_navigation)
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

if(LINGTU_NAV_CPP_BUILD_BENCHMARKS
    AND EXISTS "${LINGTU_NAV_TESTS_DIR}/benchmark_scan_planner.cpp")
  add_executable(benchmark_scan_planner
    "${LINGTU_NAV_TESTS_DIR}/benchmark_scan_planner.cpp")
  target_link_libraries(benchmark_scan_planner PRIVATE
    lingtu_nav_local_planner)
  lingtu_nav_enable_performance(benchmark_scan_planner)
endif()

add_subdirectory(endpoint)
