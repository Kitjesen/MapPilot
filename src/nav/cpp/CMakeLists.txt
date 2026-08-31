cmake_minimum_required(VERSION 3.16)
project(lingtu_nav_cpp LANGUAGES C CXX)

if(WIN32 AND NOT MSVC)
  message(FATAL_ERROR
    "LingTu navigation supports MSVC-only native Windows builds. "
    "MinGW/MSYS/clang64 toolchains can mix incompatible runtimes and GTest "
    "artifacts from the host PATH. Reconfigure from src/nav/cpp with "
    "'cmake --preset windows-x64-nav-portable --fresh'.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(cmake/NavCoreTargets.cmake)

option(LINGTU_NAV_CPP_BUILD_TESTS
  "Build focused portable navigation C++ tests" ON)
option(LINGTU_NAV_CPP_BUILD_ENDPOINT
  "Build the CycloneDDS navigation endpoint" OFF)
option(LINGTU_NAV_CPP_BUILD_BENCHMARKS
  "Build optional navigation C++ micro-benchmarks" OFF)

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

add_library(lingtu_nav_velocity_smoother STATIC
  tracking/smoother.cpp)
add_library(LingTuNav::velocity_smoother ALIAS
  lingtu_nav_velocity_smoother)
target_compile_features(lingtu_nav_velocity_smoother PUBLIC cxx_std_17)
target_include_directories(lingtu_nav_velocity_smoother PUBLIC
  "$<BUILD_INTERFACE:${LINGTU_NAV_INCLUDE_DIR}>")
target_compile_options(lingtu_nav_velocity_smoother PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
set_target_properties(lingtu_nav_velocity_smoother PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
lingtu_nav_enable_performance(lingtu_nav_velocity_smoother)

set(_LINGTU_ROLLING_PLAN_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/planning/rolling")
set(_LINGTU_ROLLING_ENDPOINT_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/endpoint/nav/runtime/rolling")
set(_LINGTU_EXPLORE_CPP_DIR
  "${LINGTU_SRC_ROOT}/explore/cpp")

if(EXISTS
      "${_LINGTU_ROLLING_PLAN_DIR}/segment.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_ENDPOINT_DIR}/lifecycle.cpp"
    AND EXISTS
      "${_LINGTU_ROLLING_ENDPOINT_DIR}/effects.cpp")
  add_library(lingtu_nav_rolling_segment STATIC
    "${_LINGTU_ROLLING_PLAN_DIR}/segment.cpp"
    "${_LINGTU_ROLLING_ENDPOINT_DIR}/lifecycle.cpp"
    "${_LINGTU_ROLLING_ENDPOINT_DIR}/effects.cpp")
  add_library(LingTuNav::rolling_segment ALIAS
    lingtu_nav_rolling_segment)
  target_compile_features(lingtu_nav_rolling_segment PUBLIC cxx_std_17)
  target_include_directories(lingtu_nav_rolling_segment PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/endpoint>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/endpoint/nav>"
    "$<BUILD_INTERFACE:${_LINGTU_EXPLORE_CPP_DIR}>")
  target_compile_options(lingtu_nav_rolling_segment PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)
  set_target_properties(lingtu_nav_rolling_segment PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
  lingtu_nav_enable_performance(lingtu_nav_rolling_segment)
endif()

if(LINGTU_NAV_CPP_BUILD_ENDPOINT)
  add_subdirectory(endpoint)
endif()

if(LINGTU_NAV_CPP_BUILD_TESTS OR LINGTU_NAV_CPP_BUILD_BENCHMARKS)
  add_subdirectory(tests)
endif()
