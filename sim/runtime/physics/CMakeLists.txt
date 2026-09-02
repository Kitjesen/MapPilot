cmake_minimum_required(VERSION 3.16)

project(lingtu_mujoco_runtime LANGUAGES CXX)

set(LINGTU_MUJOCO_TESTS_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/sim/physics")

option(LINGTU_MUJOCO_RUNTIME_BUILD_TESTS "Build MuJoCo runtime tests" ON)
option(LINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS "Build the headless smoke executable" ON)
option(LINGTU_MUJOCO_RUNTIME_BUILD_CONTRACT_TESTS "Build dependency-free runtime contract tests" ON)
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

set(LINGTU_MUJOCO_SDK_AVAILABLE TRUE)
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  set(LINGTU_MUJOCO_SDK_AVAILABLE FALSE)
  if(LINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS OR LINGTU_MUJOCO_RUNTIME_BUILD_TESTS)
    message(FATAL_ERROR
      "MuJoCo SDK was not found. Set MUJOCO_ROOT to a pinned SDK containing "
      "include/mujoco/mujoco.h and lib/mujoco, or disable the MuJoCo targets "
      "to build dependency-free contract tests."
    )
  endif()
  message(WARNING "MuJoCo SDK not found; only dependency-free contract tests will be built.")
endif()

if(LINGTU_MUJOCO_SDK_AVAILABLE)
  add_library(lingtu_mujoco_runtime STATIC
    src/mujoco_runtime.cpp
    src/physics_scene_composer.cpp
  )

  set_target_properties(lingtu_mujoco_runtime PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )

  target_include_directories(lingtu_mujoco_runtime
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" "${MUJOCO_INCLUDE_DIR}"
  )

  target_compile_features(lingtu_mujoco_runtime PUBLIC cxx_std_17)
  target_link_libraries(lingtu_mujoco_runtime PRIVATE "${MUJOCO_LIBRARY}")

  if(MSVC)
    target_compile_options(lingtu_mujoco_runtime PRIVATE /W4 /permissive-)
  else()
    target_compile_options(lingtu_mujoco_runtime PRIVATE -Wall -Wextra -Wpedantic)
  endif()
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

if(LINGTU_MUJOCO_SDK_AVAILABLE AND LINGTU_MUJOCO_RUNTIME_BUILD_HEADLESS)
  add_executable(lingtu_mujoco_headless
    apps/mujoco_headless.cpp
  )
  target_link_libraries(lingtu_mujoco_headless PRIVATE lingtu_mujoco_runtime)
  lingtu_stage_mujoco_runtime(lingtu_mujoco_headless)

  add_executable(lingtu_mujoco_snapshot
    apps/mujoco_snapshot.cpp
  )
  target_link_libraries(lingtu_mujoco_snapshot PRIVATE lingtu_mujoco_runtime)
  lingtu_stage_mujoco_runtime(lingtu_mujoco_snapshot)
endif()

if(LINGTU_MUJOCO_RUNTIME_BUILD_TESTS OR LINGTU_MUJOCO_RUNTIME_BUILD_CONTRACT_TESTS)
  enable_testing()
endif()

if(LINGTU_MUJOCO_SDK_AVAILABLE AND LINGTU_MUJOCO_RUNTIME_BUILD_TESTS)
  add_executable(test_lingtu_mujoco_runtime
    "${LINGTU_MUJOCO_TESTS_DIR}/test_mujoco_runtime.cpp"
  )
  target_link_libraries(test_lingtu_mujoco_runtime PRIVATE lingtu_mujoco_runtime)
  target_compile_definitions(test_lingtu_mujoco_runtime PRIVATE
    LINGTU_MUJOCO_TEST_MODEL="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/free_body.xml"
    LINGTU_MUJOCO_THUNDER_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../packages/robots/doso/thunder_v4/mjcf/thunderv4.xml"
    LINGTU_MUJOCO_OMNI_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/../../packages/robots/omni_cart/omni_cart.xml"
    LINGTU_MUJOCO_OPEN_FIELD="${CMAKE_CURRENT_SOURCE_DIR}/../../packages/worlds/open_field/physics/open_field.xml"
    LINGTU_MUJOCO_SCENE_MODEL="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/free_body.xml"
    LINGTU_MUJOCO_ACTUATED_MODEL="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/actuated_hinge.xml"
    LINGTU_MUJOCO_SCENE_WORLD="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/scene_world.xml"
    LINGTU_MUJOCO_STATIC_BODY_WORLD="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/scene_world_static_body.xml"
    LINGTU_MUJOCO_MID360_RAYCAST="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/mid360_raycast.xml"
    LINGTU_MUJOCO_ROBOT_GLOBAL_OPTION="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/robot_global_option.xml"
    LINGTU_MUJOCO_KINEMATIC_PROXY="${CMAKE_CURRENT_SOURCE_DIR}/../../packages/scenarios/open_field_pedestrian_crossing/physics/pedestrian_capsule.xml"
  )
  lingtu_stage_mujoco_runtime(test_lingtu_mujoco_runtime)

  add_test(NAME lingtu_mujoco_runtime COMMAND test_lingtu_mujoco_runtime)

  add_executable(test_lingtu_stable_frame_raycast
    "${LINGTU_MUJOCO_TESTS_DIR}/test_stable_frame_raycast.cpp"
  )
  target_link_libraries(test_lingtu_stable_frame_raycast PRIVATE lingtu_mujoco_runtime)
  target_compile_definitions(test_lingtu_stable_frame_raycast PRIVATE
    LINGTU_MUJOCO_STABLE_FRAME_RAYCAST="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/stable_frame_raycast.xml"
    LINGTU_MUJOCO_SCENE_WORLD="${LINGTU_MUJOCO_TESTS_DIR}/fixtures/scene_world.xml"
  )
  lingtu_stage_mujoco_runtime(test_lingtu_stable_frame_raycast)

  add_test(NAME lingtu_stable_frame_raycast COMMAND test_lingtu_stable_frame_raycast)
endif()

if(LINGTU_MUJOCO_RUNTIME_BUILD_CONTRACT_TESTS)
  find_package(Threads REQUIRED)
  add_executable(test_lingtu_runtime_contracts
    "${LINGTU_MUJOCO_TESTS_DIR}/test_runtime_contracts.cpp"
  )
  target_include_directories(test_lingtu_runtime_contracts PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_compile_features(test_lingtu_runtime_contracts PRIVATE cxx_std_17)
  target_link_libraries(test_lingtu_runtime_contracts PRIVATE Threads::Threads)
  if(MSVC)
    target_compile_options(test_lingtu_runtime_contracts PRIVATE /W4 /permissive-)
  else()
    target_compile_options(test_lingtu_runtime_contracts PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME lingtu_runtime_contracts COMMAND test_lingtu_runtime_contracts)
endif()
