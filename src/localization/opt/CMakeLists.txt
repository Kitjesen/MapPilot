cmake_minimum_required(VERSION 3.20)
project(map_opt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(LINGTU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(PGO_KERNEL_DIR "${LINGTU_ROOT}/src/kernels/slam/pose_graph_opt")
set(LINGTU_POSE_GRAPH_OPT_LIBRARY "" CACHE FILEPATH
    "Optional prebuilt pose_graph_opt static library")
if(LINGTU_POSE_GRAPH_OPT_LIBRARY)
  get_filename_component(PGO_KERNEL_LIBRARY "${LINGTU_POSE_GRAPH_OPT_LIBRARY}" ABSOLUTE)
  if(NOT EXISTS "${PGO_KERNEL_LIBRARY}")
    message(FATAL_ERROR "LINGTU_POSE_GRAPH_OPT_LIBRARY does not exist: ${PGO_KERNEL_LIBRARY}")
  endif()
else()
  find_program(CARGO_EXECUTABLE cargo REQUIRED)
  set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
  if(WIN32)
    set(PGO_KERNEL_LIBRARY "${CARGO_TARGET_DIR}/release/pose_graph_opt.lib")
  else()
    set(PGO_KERNEL_LIBRARY "${CARGO_TARGET_DIR}/release/libpose_graph_opt.a")
  endif()

  add_custom_command(
    OUTPUT "${PGO_KERNEL_LIBRARY}"
    COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${CARGO_TARGET_DIR}"
            "${CARGO_EXECUTABLE}" build --release --manifest-path "${PGO_KERNEL_DIR}/Cargo.toml"
    DEPENDS
      "${PGO_KERNEL_DIR}/Cargo.toml"
      "${PGO_KERNEL_DIR}/Cargo.lock"
      "${PGO_KERNEL_DIR}/src/ffi.rs"
      "${PGO_KERNEL_DIR}/src/geometry.rs"
      "${PGO_KERNEL_DIR}/src/lib.rs"
      "${PGO_KERNEL_DIR}/src/solver.rs"
    VERBATIM)
  add_custom_target(pose_graph_opt_build DEPENDS "${PGO_KERNEL_LIBRARY}")
endif()

add_library(pose_graph_opt STATIC IMPORTED GLOBAL)
set_target_properties(pose_graph_opt PROPERTIES IMPORTED_LOCATION "${PGO_KERNEL_LIBRARY}")
if(TARGET pose_graph_opt_build)
  add_dependencies(pose_graph_opt pose_graph_opt_build)
endif()

add_library(map_opt STATIC
  assembly.cpp
  constraints.cpp
  graph.cpp
  loop_constraints.cpp
  map.cpp
  pgo.cpp)
target_include_directories(map_opt
  PUBLIC "${LINGTU_ROOT}/src" "${PGO_KERNEL_DIR}/include")
target_compile_definitions(map_opt PUBLIC LT_POSE_GRAPH_OPT_STATIC)
target_link_libraries(map_opt PUBLIC pose_graph_opt)
if(WIN32)
  target_link_libraries(map_opt PUBLIC ws2_32 userenv bcrypt ntdll)
elseif(UNIX)
  target_link_libraries(map_opt PUBLIC dl pthread m)
endif()

add_executable(lt_pgo pgo_cli.cpp)
target_link_libraries(lt_pgo PRIVATE map_opt)

add_executable(lt_loop_verify loop_cli.cpp)
target_link_libraries(lt_loop_verify PRIVATE map_opt)

include(CTest)
if(BUILD_TESTING)
  add_executable(pgo_test tests/pgo_test.cpp)
  target_link_libraries(pgo_test PRIVATE map_opt)
  add_test(NAME pgo_core COMMAND pgo_test)
  add_executable(loop_information_test tests/loop_information_test.cpp)
  target_link_libraries(loop_information_test PRIVATE map_opt)
  add_test(NAME loop_information COMMAND loop_information_test)
  add_executable(constraint_assembly_test tests/constraint_assembly_test.cpp)
  target_link_libraries(constraint_assembly_test PRIVATE map_opt)
  add_dependencies(constraint_assembly_test lt_pgo)
  add_test(NAME constraint_assembly
           COMMAND constraint_assembly_test $<TARGET_FILE:lt_pgo>)
endif()
