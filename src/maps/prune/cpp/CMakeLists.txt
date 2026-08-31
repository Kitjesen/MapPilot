cmake_minimum_required(VERSION 3.16)
project(lingtu_prune_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LINGTU_PRUNE_ERASOR2
       "Build optional GPLv3 ERASOR2 reference backend from third_party/research_nav/ERASOR2"
       OFF)
option(LINGTU_ERASOR2_USE_RERUN_STUB
       "Use LingTu's no-op rerun_sdk stub for offline ERASOR2 robot builds" ON)

add_library(prune_core STATIC
  core/evidence.cpp
  core/flow.cpp
  core/io.cpp
  core/save.cpp
  core/score.cpp
  core/text.cpp
  cleaner.cpp)

target_include_directories(prune_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(prune
  prune.cpp)

target_link_libraries(prune PRIVATE
  prune_core)

if(LINGTU_PRUNE_ERASOR2)
  add_library(erasor2_stage_core STATIC
    refs/erasor2/stager.cpp)
  target_include_directories(erasor2_stage_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/refs/erasor2"
    "${CMAKE_CURRENT_SOURCE_DIR}")

  add_executable(erasor2_stage
    refs/erasor2/stage.cpp)
  target_link_libraries(erasor2_stage PRIVATE
    erasor2_stage_core)

  get_filename_component(LINGTU_REPO_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
  set(ERASOR2_SOURCE_DIR
    "${LINGTU_REPO_ROOT}/third_party/research_nav/ERASOR2"
    CACHE PATH "Path to the GPLv3 ERASOR2 checkout")
  if(NOT EXISTS "${ERASOR2_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "LINGTU_PRUNE_ERASOR2=ON but ERASOR2_SOURCE_DIR is missing: "
      "${ERASOR2_SOURCE_DIR}")
  endif()
  if(LINGTU_ERASOR2_USE_RERUN_STUB)
    set(FETCHCONTENT_SOURCE_DIR_RERUN_SDK
      "${CMAKE_CURRENT_SOURCE_DIR}/refs/erasor2/rerun_stub"
      CACHE PATH "LingTu no-op rerun_sdk stub" FORCE)
  endif()
  add_subdirectory("${ERASOR2_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/erasor2_upstream")
  if(TARGET run_erasor2)
    set_target_properties(run_erasor2 PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  add_executable(erasor2_clean
    refs/erasor2/clean.cpp)
  add_dependencies(erasor2_clean
    erasor2_stage
    run_erasor2)
endif()

if(MSVC)
  target_compile_options(prune_core PRIVATE /W4 /permissive-)
  target_compile_options(prune PRIVATE /W4 /permissive-)
  if(LINGTU_PRUNE_ERASOR2)
    target_compile_options(erasor2_stage_core PRIVATE /W4 /permissive-)
    target_compile_options(erasor2_stage PRIVATE /W4 /permissive-)
    target_compile_options(erasor2_clean PRIVATE /W4 /permissive-)
  endif()
else()
  target_compile_options(prune_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(prune PRIVATE -Wall -Wextra -Wpedantic)
  if(LINGTU_PRUNE_ERASOR2)
    target_compile_options(erasor2_stage_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(erasor2_stage PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(erasor2_clean PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

include(CTest)
if(BUILD_TESTING)
  add_test(NAME prune_help COMMAND prune --help)
  if(LINGTU_PRUNE_ERASOR2)
    add_test(NAME erasor2_stage_help COMMAND erasor2_stage --help)
    add_test(NAME erasor2_clean_help COMMAND erasor2_clean --help)
  endif()
endif()
