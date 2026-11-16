# Test and benchmark targets are owned by this directory.
if(LINGTU_NAV_CPP_BUILD_TESTS)
  add_executable(test_body_contact "${LINGTU_NAV_TESTS_DIR}/test_body_contact.cpp")
  target_link_libraries(test_body_contact PRIVATE lingtu_nav_body)
  add_test(NAME test_body_contact COMMAND test_body_contact)
  add_executable(test_platform_runtime
    "${LINGTU_NAV_TESTS_DIR}/test_platform_runtime.cpp")
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
      if(target STREQUAL "test_scan_planner")
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
            AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "9"
            AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "10"
            AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
          # GCC 9/aarch64 ICEs in add_symbol_to_partition_1 for this test.
          # Keep LTO, bypass its failing partition phase; production IPO stays on.
          target_link_options(${target} PRIVATE -flto-partition=none)
        endif()
        gtest_discover_tests(${target} PROPERTIES TIMEOUT 120)
      else()
        gtest_discover_tests(${target})
      endif()
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
        test_scan_polynomial
        test_scan_search_metrics
        test_scan_elevation
        test_scan_refine_solver
        test_scan_body
        test_body_terrain
        test_scan_spline_velocity_bounds
        test_local_plan_task
        test_recovery_rotation
        test_recovery_sequence
        test_local_planner_core)
      lingtu_add_nav_gtest(${test_name} "${test_name}.cpp")
    endforeach()

    target_compile_definitions(test_scan_body PRIVATE
      LINGTU_TEST_THUNDER_BODY="${LINGTU_SRC_ROOT}/../config/robots/doso/thunder_v4/body/thunderv4_navigation.body2")

    lingtu_add_nav_gtest(test_scan_failure_snapshot test_scan_failure_snapshot.cpp)
    target_sources(test_scan_failure_snapshot PRIVATE
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/nav_status_writer.cpp"
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/status/status_snapshot_file_writer.cpp"
      "${LINGTU_SRC_ROOT}/maps/cpp/io/json.cpp")
    target_include_directories(test_scan_failure_snapshot PRIVATE
      "${LINGTU_SRC_ROOT}"
      "${LINGTU_SRC_ROOT}/maps/include"
      "${LINGTU_NAV_CPP_ROOT}/endpoint"
      "${LINGTU_NAV_CPP_ROOT}/endpoint/nav")

    target_include_directories(test_terrain_core PRIVATE
      "${LINGTU_SRC_ROOT}/maps/include")
    target_include_directories(test_benchmark PRIVATE
      "${LINGTU_SRC_ROOT}/maps/include")

    target_link_libraries(test_far_planner PRIVATE lingtu_nav_far)
    target_link_libraries(test_far_c_api PRIVATE lingtu_nav_far_c_api)
    target_link_libraries(test_recovery_rotation PRIVATE lingtu_nav_navigation)
    target_link_libraries(test_local_planner_core PRIVATE lingtu_nav_navigation)
    set_target_properties(test_far_c_api PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

    lingtu_add_nav_gtest(test_executor test_executor.cpp)
    target_compile_definitions(test_executor PRIVATE
      LINGTU_TEST_PATH_LIBRARY="${LINGTU_NAV_ROOT}/cpp/planning/local/cmu/paths/go2")
    target_link_libraries(test_executor PRIVATE lingtu_nav_navigation)
    lingtu_add_nav_gtest(test_pose_route test_pose_route.cpp)
    target_link_libraries(test_pose_route PRIVATE lingtu_nav_navigation)
    lingtu_add_nav_gtest(test_stairs test_stairs.cpp)
    target_link_libraries(test_stairs PRIVATE lingtu_nav_navigation)
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

if(LINGTU_NAV_CPP_BUILD_BENCHMARKS)
  add_executable(benchmark_scan_braking
    "${LINGTU_NAV_TESTS_DIR}/benchmark_scan_braking.cpp")
  target_link_libraries(benchmark_scan_braking PRIVATE
    lingtu_nav_local_planner)
  lingtu_nav_enable_performance(benchmark_scan_braking)
endif()

option(LINGTU_NAV_CPP_BUILD_SAVED_ROUTE_BENCHMARK
  "Build the saved-map component probe with native OctoMap dependencies" OFF)

if(LINGTU_NAV_CPP_BUILD_BENCHMARKS
    AND LINGTU_NAV_CPP_BUILD_SAVED_ROUTE_BENCHMARK
    AND NOT TARGET octoplanner3d_runtime)
  add_subdirectory(
    "${LINGTU_NAV_CPP_ROOT}/planning/global/octoplanner"
    "${CMAKE_BINARY_DIR}/saved_route_octoplanner" EXCLUDE_FROM_ALL)
endif()

if(LINGTU_NAV_CPP_BUILD_BENCHMARKS
    AND LINGTU_NAV_CPP_BUILD_SAVED_ROUTE_BENCHMARK)
  add_executable(benchmark_saved_route
    "${LINGTU_NAV_TESTS_DIR}/benchmark_saved_route.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/app/config/parse.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/app/config/build.cpp")
  target_include_directories(benchmark_saved_route PRIVATE
    "${LINGTU_NAV_TESTS_DIR}"
    "${LINGTU_NAV_CPP_ROOT}"
    "${LINGTU_NAV_CPP_ROOT}/include"
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_SRC_ROOT}/explore/cpp"
    "${LINGTU_SRC_ROOT}/maps/include")
  target_link_libraries(benchmark_saved_route PRIVATE
    lingtu_nav_navigation lingtu_nav_far octoplanner3d_runtime
    octoplanner3d_global_planner octomap_vendor octomath_vendor)
  target_compile_options(benchmark_saved_route PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
  lingtu_nav_enable_performance(benchmark_saved_route)

  add_executable(benchmark_body_execution
    "${LINGTU_NAV_TESTS_DIR}/benchmark_body_execution.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/app/config/parse.cpp"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav/app/config/build.cpp")
  target_include_directories(benchmark_body_execution PRIVATE
    "${LINGTU_NAV_CPP_ROOT}/endpoint"
    "${LINGTU_NAV_CPP_ROOT}/endpoint/nav"
    "${LINGTU_SRC_ROOT}"
    "${LINGTU_SRC_ROOT}/explore/cpp"
    "${LINGTU_SRC_ROOT}/maps/include")
  target_compile_options(benchmark_body_execution PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
  target_link_libraries(benchmark_body_execution PRIVATE
    lingtu_nav_navigation lingtu_nav_far octoplanner3d_runtime
    octoplanner3d_global_planner octomap_vendor octomath_vendor)
  lingtu_nav_enable_performance(benchmark_body_execution)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS AND TARGET GTest::gtest_main)
  lingtu_add_nav_gtest(test_body_support_map test_body_support_map.cpp)
  target_compile_definitions(test_body_support_map PRIVATE
    LINGTU_TEST_THUNDER_BODY="${LINGTU_SRC_ROOT}/../config/robots/doso/thunder_v4/body/thunderv4_navigation.body2")
endif()
