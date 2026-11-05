cmake_minimum_required(VERSION 3.16)
project(lingtu_prune_cpp LANGUAGES CXX)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(prune_core STATIC
  core/evidence.cpp
  core/visibility.cpp
  core/io.cpp
  core/save.cpp
  core/text.cpp
  cleaner.cpp)

target_include_directories(prune_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(prune_core PRIVATE cxx_std_20)
add_executable(prune
  prune.cpp)

target_link_libraries(prune PRIVATE
  prune_core)

install(TARGETS prune
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)

if(MSVC)
  target_compile_options(prune_core PRIVATE /W4 /permissive-)
  target_compile_options(prune PRIVATE /W4 /permissive-)
else()
  target_compile_options(prune_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(prune PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  include(CTest)
endif()
if(BUILD_TESTING AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  add_test(NAME prune_help COMMAND prune --help)
  add_executable(prune_test "${CMAKE_CURRENT_SOURCE_DIR}/../../../../tests/maps/cpp/prune_test.cpp")
  target_link_libraries(prune_test PRIVATE prune_core)
  foreach(test_case report dry_run empty_apply repeat_apply output_aliases truncated_source bad_threshold
      visibility endpoint_confirmation local_ground calibrated_origin ground mixed_ground pose_order
      missing_origin surface expired_surface)
    add_test(NAME prune_${test_case} COMMAND prune_test ${test_case})
  endforeach()
endif()
