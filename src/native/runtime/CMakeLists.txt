cmake_minimum_required(VERSION 3.16)

project(lingtu_native_runtime_components LANGUAGES CXX)

option(LINGTU_NATIVE_RUNTIME_COMPONENTS_BUILD_TESTS "Build native runtime manifest tests" ON)
set(LINGTU_POSE_GRAPH_OPT_LIB "" CACHE FILEPATH "Prebuilt pose_graph_opt static library")

set(POSE_GRAPH_OPT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../kernels/slam/pose_graph_opt")
if(LINGTU_POSE_GRAPH_OPT_LIB)
  set(POSE_GRAPH_OPT_LIB "${LINGTU_POSE_GRAPH_OPT_LIB}")
elseif(WIN32)
  set(POSE_GRAPH_OPT_LIB "${POSE_GRAPH_OPT_DIR}/target/release/lingtu_pose_graph_opt.lib")
else()
  set(POSE_GRAPH_OPT_LIB "${POSE_GRAPH_OPT_DIR}/target/release/liblingtu_pose_graph_opt.a")
endif()

if(NOT EXISTS "${POSE_GRAPH_OPT_LIB}")
  find_program(CARGO_EXECUTABLE cargo)
  if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR
      "pose_graph_opt static library is missing and cargo is unavailable. "
      "Pass -DLINGTU_POSE_GRAPH_OPT_LIB=/path/to/liblingtu_pose_graph_opt.a "
      "or build src/kernels/slam/pose_graph_opt first."
    )
  endif()

  file(GLOB_RECURSE POSE_GRAPH_OPT_RS
    "${POSE_GRAPH_OPT_DIR}/src/*.rs"
  )

  add_custom_command(
    OUTPUT "${POSE_GRAPH_OPT_LIB}"
    COMMAND "${CARGO_EXECUTABLE}" build --release --manifest-path "${POSE_GRAPH_OPT_DIR}/Cargo.toml"
    DEPENDS
      "${POSE_GRAPH_OPT_DIR}/Cargo.toml"
      ${POSE_GRAPH_OPT_RS}
    WORKING_DIRECTORY "${POSE_GRAPH_OPT_DIR}"
    VERBATIM
  )

  add_custom_target(lt_pose_graph_opt_build DEPENDS "${POSE_GRAPH_OPT_LIB}")
else()
  add_custom_target(lt_pose_graph_opt_build)
endif()

add_library(lt_pose_graph_opt STATIC IMPORTED GLOBAL)
set_target_properties(lt_pose_graph_opt PROPERTIES
  IMPORTED_LOCATION "${POSE_GRAPH_OPT_LIB}"
)
add_dependencies(lt_pose_graph_opt lt_pose_graph_opt_build)

add_library(lt_native_runtime STATIC
  components.cpp
  ../../localization/opt/map.cpp
  ../../localization/opt/graph.cpp
  ../../localization/opt/loop_constraints.cpp
  ../../localization/opt/pgo.cpp
  ../../localization/opt/hba.cpp
)

target_include_directories(lt_native_runtime
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    "${POSE_GRAPH_OPT_DIR}/include"
)

target_compile_features(lt_native_runtime PUBLIC cxx_std_17)
target_compile_definitions(lt_native_runtime PRIVATE LT_POSE_GRAPH_OPT_STATIC)

target_link_libraries(lt_native_runtime
  PRIVATE lt_pose_graph_opt
)

if(WIN32)
  if(TARGET windows)
    target_link_libraries(lt_native_runtime PRIVATE windows)
  endif()
  target_link_libraries(
    lt_native_runtime
    PRIVATE kernel32 ntdll userenv ws2_32 dbghelp bcrypt advapi32
  )
elseif(UNIX)
  target_link_libraries(lt_native_runtime PRIVATE pthread dl m)
endif()

add_executable(lt_native
  components_cli.cpp
)

target_link_libraries(lt_native
  PRIVATE lt_native_runtime
)

add_executable(lt_pgo
  ../../localization/opt/cli.cpp
)

target_link_libraries(lt_pgo
  PRIVATE lt_native_runtime
)

add_executable(lt_hba
  ../../localization/opt/cli.cpp
)

target_link_libraries(lt_hba
  PRIVATE lt_native_runtime
)

add_executable(lt_loop_verify
  ../../localization/opt/loop_cli.cpp
)

target_link_libraries(lt_loop_verify
  PRIVATE lt_native_runtime
)

if(LINGTU_NATIVE_RUNTIME_COMPONENTS_BUILD_TESTS)
  enable_testing()

  add_executable(test_native_runtime
    test_components.cpp
  )

  target_link_libraries(test_native_runtime
    PRIVATE lt_native_runtime
  )

  add_test(NAME native_runtime COMMAND test_native_runtime)

  add_executable(test_loop_constraints
    test_loop_constraints.cpp
  )

  target_link_libraries(test_loop_constraints
    PRIVATE lt_native_runtime
  )

  add_test(NAME loop_constraints COMMAND test_loop_constraints)
endif()
