cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_plan_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LINGTU_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../..")
set(NAV_KERNEL_DIR "${LINGTU_ROOT}/src/nav/kernel")
set(LOCAL_PLANNER_CPP_DIR "${LINGTU_ROOT}/src/nav/services/plan/local_planner/cpp")

add_library(lingtu_nav_plan_cpp STATIC
  nav_loop.cpp
  "${NAV_KERNEL_DIR}/src/path_follower_core.cpp")

target_include_directories(lingtu_nav_plan_cpp
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${NAV_KERNEL_DIR}/include"
    "${LOCAL_PLANNER_CPP_DIR}")

target_compile_options(lingtu_nav_plan_cpp PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)

option(LINGTU_NAV_PLAN_CPP_BUILD_TESTS
  "Build standalone C++ nav plan chain tests" OFF)

if(LINGTU_NAV_PLAN_CPP_BUILD_TESTS)
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

  add_executable(test_nav_loop
    tests/test_nav_loop.cpp)
  target_compile_definitions(test_nav_loop
    PRIVATE
      LINGTU_TEST_PATH_LIBRARY="${LINGTU_ROOT}/src/nav/services/plan/local_planner/paths")
  target_link_libraries(test_nav_loop
    PRIVATE lingtu_nav_plan_cpp GTest::gtest_main)
  gtest_discover_tests(test_nav_loop)
endif()
