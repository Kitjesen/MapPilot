cmake_minimum_required(VERSION 3.16)

project(lingtu_mujoco_runtime LANGUAGES CXX)

option(LINGTU_MUJOCO_RUNTIME_BUILD_TESTS "Build MuJoCo runtime tests" ON)
option(LINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS "Build the headless smoke executable" ON)
set(MUJOCO_ROOT "" CACHE PATH "MuJoCo SDK root containing include/, lib/, and bin/")

find_path(
  MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS "${MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}"
  PATH_SUFFIXES include
)

find_library(
  MUJOCO_LIBRARY
  NAMES mujoco
  HINTS "${MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}"
  PATH_SUFFIXES lib
)

if(WIN32)
  find_file(
    MUJOCO_RUNTIME_LIBRARY
    NAMES mujoco.dll
    HINTS "${MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}"
    PATH_SUFFIXES bin
  )
elseif(APPLE)
  find_file(
    MUJOCO_RUNTIME_LIBRARY
    NAMES libmujoco.dylib
    HINTS "${MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}"
    PATH_SUFFIXES lib bin
  )
else()
  find_file(
    MUJOCO_RUNTIME_LIBRARY
    NAMES libmujoco.so
    HINTS "${MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}"
    PATH_SUFFIXES lib bin
  )
endif()

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo SDK was not found. Set MUJOCO_ROOT to a pinned SDK containing "
    "include/mujoco/mujoco.h and lib/mujoco."
  )
endif()

add_library(lingtu_mujoco_runtime STATIC
  src/mujoco_runtime.cpp
)

set_target_properties(lingtu_mujoco_runtime PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(lingtu_mujoco_runtime
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${MUJOCO_INCLUDE_DIR}"
)

target_compile_features(lingtu_mujoco_runtime PUBLIC cxx_std_17)
target_link_libraries(lingtu_mujoco_runtime PRIVATE "${MUJOCO_LIBRARY}")

if(MSVC)
  target_compile_options(lingtu_mujoco_runtime PRIVATE /W4 /permissive-)
else()
  target_compile_options(lingtu_mujoco_runtime PRIVATE -Wall -Wextra -Wpedantic)
endif()

function(lingtu_stage_mujoco_runtime target)
  if(MUJOCO_RUNTIME_LIBRARY)
    add_custom_command(
      TARGET "${target}"
      POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
              "${MUJOCO_RUNTIME_LIBRARY}"
              "$<TARGET_FILE_DIR:${target}>"
      VERBATIM
    )
  endif()
endfunction()

if(LINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS)
  add_executable(lingtu_mujoco_headless
    apps/mujoco_headless.cpp
  )
  target_link_libraries(lingtu_mujoco_headless PRIVATE lingtu_mujoco_runtime)
  lingtu_stage_mujoco_runtime(lingtu_mujoco_headless)
endif()

if(LINGTU_MUJOCO_RUNTIME_BUILD_TESTS)
  enable_testing()

  add_executable(test_lingtu_mujoco_runtime
    tests/test_mujoco_runtime.cpp
  )
  target_link_libraries(test_lingtu_mujoco_runtime PRIVATE lingtu_mujoco_runtime)
  target_compile_definitions(test_lingtu_mujoco_runtime PRIVATE
    LINGTU_MUJOCO_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/free_body.xml"
    LINGTU_MUJOCO_THUNDER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../robots/thunderv4/mjcf/thunderv4.xml"
  )
  lingtu_stage_mujoco_runtime(test_lingtu_mujoco_runtime)

  add_test(NAME lingtu_mujoco_runtime COMMAND test_lingtu_mujoco_runtime)
endif()
