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
