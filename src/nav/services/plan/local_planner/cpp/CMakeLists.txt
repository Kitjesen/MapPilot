cmake_minimum_required(VERSION 3.14)
project(local_planner_cpp LANGUAGES CXX)

set(NAV_KERNEL_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../kernel/include")

add_library(local_planner_cpp INTERFACE)
target_compile_features(local_planner_cpp INTERFACE cxx_std_17)
target_include_directories(local_planner_cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${NAV_KERNEL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/services/plan/local_planner/cpp>)
target_compile_options(local_planner_cpp INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

option(LOCAL_PLANNER_CPP_BUILD_TESTS
  "Build standalone local planner C++ tests" OFF)
option(LOCAL_PLANNER_CPP_BUILD_BENCHMARKS
  "Build standalone local planner C++ micro-benchmarks" OFF)

find_package(OpenMP QUIET)

if(LOCAL_PLANNER_CPP_BUILD_TESTS)
  find_package(GTest CONFIG QUIET)
  if(NOT TARGET GTest::gtest_main)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.14.0)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  enable_testing()
  include(GoogleTest)

  add_executable(test_local_planner_core
    tests/test_local_planner_core.cpp)
  target_link_libraries(test_local_planner_core
    PRIVATE local_planner_cpp GTest::gtest_main)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_local_planner_core PRIVATE OpenMP::OpenMP_CXX)
  endif()
  gtest_discover_tests(test_local_planner_core)
endif()

if(LOCAL_PLANNER_CPP_BUILD_BENCHMARKS)
  add_executable(benchmark_local_planner_core
    tests/benchmark_local_planner_core.cpp)
  target_link_libraries(benchmark_local_planner_core
    PRIVATE local_planner_cpp)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(benchmark_local_planner_core PRIVATE OpenMP::OpenMP_CXX)
  endif()
endif()

install(FILES
  local_planner.hpp
  local_planner_scoring.hpp
  DESTINATION include/services/plan/local_planner/cpp
)
