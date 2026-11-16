cmake_minimum_required(VERSION 3.16)
project(octoplanner3d_runtime LANGUAGES C CXX)
include(CTest)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

get_filename_component(
  OCTOPLANNER3D_DEFAULT_SOURCE_DIR
  "${CMAKE_CURRENT_LIST_DIR}/vendor"
  ABSOLUTE)
set(OCTOPLANNER3D_SOURCE_DIR
  "${OCTOPLANNER3D_DEFAULT_SOURCE_DIR}"
  CACHE PATH "Path to the local OctoPlanner3D source")
get_filename_component(LINGTU_NAV_CPP_DIR
  "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${LINGTU_NAV_CPP_DIR}/../../../scripts/build/cmake/settings.cmake")
option(OCTOPLANNER3D_REQUIRE_PCL "Require the PCL-backed octoplanner3d_pcd_to_octomap target" OFF)
option(
  OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT
  "Link PCL-based direct .pcd loading into the online planner runtime"
  OFF)

if(NOT EXISTS "${OCTOPLANNER3D_SOURCE_DIR}/planner/include/global_planner.h")
  message(FATAL_ERROR "OCTOPLANNER3D_SOURCE_DIR must point to a checkout with planner/include/global_planner.h: ${OCTOPLANNER3D_SOURCE_DIR}")
endif()

find_package(PCL QUIET COMPONENTS common io octree)
find_path(OCTOMAP_SYSTEM_INCLUDE_DIR NAMES octomap/OcTree.h)
find_library(OCTOMAP_SYSTEM_LIBRARY NAMES octomap)
find_library(OCTOMATH_SYSTEM_LIBRARY NAMES octomath)
if(NOT OCTOMAP_SYSTEM_INCLUDE_DIR OR
   NOT OCTOMAP_SYSTEM_LIBRARY OR
   NOT OCTOMATH_SYSTEM_LIBRARY)
  message(FATAL_ERROR
    "OctoPlanner3D requires a target-native OctoMap installation. "
    "Install liboctomap-dev; precompiled vendored .so files are not accepted.")
endif()
set(OCTOMAP_INCLUDE_DIR "${OCTOMAP_SYSTEM_INCLUDE_DIR}")
set(OCTOMAP_LIBRARY "${OCTOMAP_SYSTEM_LIBRARY}")
set(OCTOMATH_LIBRARY "${OCTOMATH_SYSTEM_LIBRARY}")
message(STATUS "Using system OctoMap: ${OCTOMAP_LIBRARY}")
set(OCTOPLANNER3D_PCD_CONVERTER_HEADER "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include/pcd2octomap_converter.h")
set(OCTOPLANNER3D_PCD_CONVERTER_SOURCE "${OCTOPLANNER3D_SOURCE_DIR}/octomap/src/pcd2octomap_converter.cpp")
set(OCTOPLANNER3D_ENABLE_PCD OFF)
if(PCL_FOUND)
  if(EXISTS "${OCTOPLANNER3D_PCD_CONVERTER_HEADER}" AND EXISTS "${OCTOPLANNER3D_PCD_CONVERTER_SOURCE}")
    set(OCTOPLANNER3D_ENABLE_PCD ON)
    message(STATUS "PCL found: enabling the standalone .pcd -> OctoMap converter")
  elseif(OCTOPLANNER3D_REQUIRE_PCL)
    message(FATAL_ERROR "OCTOPLANNER3D_REQUIRE_PCL=ON but OctoPlanner3D PCD converter source/header is missing")
  else()
    message(WARNING "PCL found but OctoPlanner3D PCD converter source/header missing: building octoplanner3d_headless with .bt OctoMap input support only")
  endif()
elseif(OCTOPLANNER3D_REQUIRE_PCL)
  message(FATAL_ERROR "OCTOPLANNER3D_REQUIRE_PCL=ON but PCL common/io/octree was not found")
else()
  message(WARNING "PCL not found: building octoplanner3d_headless with .bt OctoMap input support only")
endif()

if(WIN32)
  add_library(octomap_vendor STATIC IMPORTED GLOBAL)
  add_library(octomath_vendor STATIC IMPORTED GLOBAL)
  set_target_properties(octomap_vendor PROPERTIES
    IMPORTED_LOCATION "${OCTOMAP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIR}"
  )
  set_target_properties(octomath_vendor PROPERTIES
    IMPORTED_LOCATION "${OCTOMATH_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIR}"
  )
else()
  add_library(octomap_vendor SHARED IMPORTED GLOBAL)
  set_target_properties(octomap_vendor PROPERTIES
    IMPORTED_LOCATION "${OCTOMAP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIR}"
  )

  add_library(octomath_vendor SHARED IMPORTED GLOBAL)
  set_target_properties(octomath_vendor PROPERTIES
    IMPORTED_LOCATION "${OCTOMATH_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIR}"
  )
endif()

set(LINGTU_MLS_CORE_DIR "${LINGTU_NAV_CPP_DIR}/../../kernels/nav/mls_core")
find_program(CARGO_EXECUTABLE cargo REQUIRED)
set(LINGTU_MLS_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo-mls")
if(WIN32)
  set(LINGTU_MLS_LIBRARY "${LINGTU_MLS_TARGET_DIR}/release/lingtu_mls_core.lib")
else()
  set(LINGTU_MLS_LIBRARY "${LINGTU_MLS_TARGET_DIR}/release/liblingtu_mls_core.a")
endif()
add_custom_command(
  OUTPUT "${LINGTU_MLS_LIBRARY}"
  COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${LINGTU_MLS_TARGET_DIR}"
          "${CARGO_EXECUTABLE}" build --release --manifest-path "${LINGTU_MLS_CORE_DIR}/Cargo.toml"
  DEPENDS
    "${LINGTU_MLS_CORE_DIR}/Cargo.toml"
    "${LINGTU_MLS_CORE_DIR}/src/lib.rs"
    "${LINGTU_MLS_CORE_DIR}/src/adjacency.rs"
    "${LINGTU_MLS_CORE_DIR}/src/corridor.rs"
    "${LINGTU_MLS_CORE_DIR}/src/edge_repair.rs"
    "${LINGTU_MLS_CORE_DIR}/src/edges.rs"
    "${LINGTU_MLS_CORE_DIR}/src/nodes.rs"
    "${LINGTU_MLS_CORE_DIR}/src/node_relocation.rs"
    "${LINGTU_MLS_CORE_DIR}/src/surfaces.rs"
    "${LINGTU_MLS_CORE_DIR}/src/stable_graph.rs"
    "${LINGTU_MLS_CORE_DIR}/src/voronoi_repair.rs"
    "${LINGTU_MLS_CORE_DIR}/include/lingtu_mls_core.h"
  VERBATIM)
add_custom_target(lingtu_mls_core_build DEPENDS "${LINGTU_MLS_LIBRARY}")
add_library(lingtu_mls_core STATIC IMPORTED GLOBAL)
set_target_properties(lingtu_mls_core PROPERTIES
  IMPORTED_LOCATION "${LINGTU_MLS_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${LINGTU_MLS_CORE_DIR}/include")
if(WIN32)
  set_property(TARGET lingtu_mls_core APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES "ws2_32;userenv;bcrypt;ntdll")
elseif(UNIX)
  set_property(TARGET lingtu_mls_core APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES "dl;pthread;m")
endif()
add_dependencies(lingtu_mls_core lingtu_mls_core_build)

add_library(octoplanner3d_global_planner STATIC
  "${OCTOPLANNER3D_SOURCE_DIR}/planner/src/global_planner.cpp"
  mls_backend.cpp
  route_smoothing.cpp
)
# Raw occupancy stays in OctoMap. Rust MLS owns prepared surfaces and the sparse graph.
# Geometry rejection can request one bounded Rust detour before the grid fallback.
target_include_directories(octoplanner3d_global_planner
  PUBLIC
    "${LINGTU_NAV_CPP_DIR}/../../maps/include"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${OCTOPLANNER3D_SOURCE_DIR}/planner/include"
    "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
)
target_link_libraries(octoplanner3d_global_planner
  PUBLIC lingtu_mls_core
  PUBLIC "$<BUILD_INTERFACE:lingtu_settings>"
  PUBLIC
    octomap_vendor
    octomath_vendor
)

if(OCTOPLANNER3D_ENABLE_PCD)
  add_library(octoplanner3d_pcd_converter STATIC
    "${OCTOPLANNER3D_PCD_CONVERTER_SOURCE}"
  )
  target_include_directories(octoplanner3d_pcd_converter
    PUBLIC
      "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
      ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(octoplanner3d_pcd_converter
    PUBLIC
      octomap_vendor
      octomath_vendor
      ${PCL_LIBRARIES}
  )
endif()

if(NOT TARGET lingtu_nav_body)
  add_library(lingtu_nav_body STATIC
    "${LINGTU_NAV_CPP_DIR}/collision/body.cpp"
    "${LINGTU_NAV_CPP_DIR}/collision/body_io.cpp"
    "${LINGTU_NAV_CPP_DIR}/collision/terrain.cpp")
  target_include_directories(lingtu_nav_body PUBLIC "${LINGTU_NAV_CPP_DIR}")
endif()

add_library(octoplanner3d_runtime STATIC
  octoplanner3d_core.cpp
  prepared.cpp
)
target_include_directories(octoplanner3d_runtime
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${LINGTU_NAV_CPP_DIR}"
)
target_link_libraries(octoplanner3d_runtime
  PUBLIC
    "$<BUILD_INTERFACE:lingtu_settings>"
  PRIVATE
    octoplanner3d_global_planner
)
if(TARGET lingtu_maps_artifacts)
  target_link_libraries(octoplanner3d_runtime PRIVATE lingtu_maps_artifacts)
else()
  target_sources(octoplanner3d_runtime PRIVATE
    "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/storage/no_go.cpp"
    "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/io/json.cpp")
endif()
if(OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT AND NOT OCTOPLANNER3D_ENABLE_PCD)
  message(FATAL_ERROR
    "OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT=ON requires the PCL-backed converter")
endif()

if(OCTOPLANNER3D_RUNTIME_ENABLE_PCD_INPUT)
  target_compile_definitions(octoplanner3d_runtime PRIVATE OCTOPLANNER3D_ENABLE_PCD=1)
  target_link_libraries(octoplanner3d_runtime PRIVATE octoplanner3d_pcd_converter)
endif()

add_executable(octoplanner3d_headless octoplanner3d_headless.cpp)
target_link_libraries(octoplanner3d_headless
  PRIVATE
    octoplanner3d_runtime
)
set_target_properties(octoplanner3d_headless PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(octoplanner3d_edit_octomap edit_octomap.cpp)
if(UNIX)
  target_link_options(octoplanner3d_edit_octomap PRIVATE "-Wl,--no-as-needed")
endif()
target_link_libraries(octoplanner3d_edit_octomap
  PRIVATE
    octomap_vendor
    octomath_vendor
)
set_target_properties(octoplanner3d_edit_octomap PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(octoplanner3d_dump_octomap dump_octomap.cpp)
target_link_libraries(octoplanner3d_dump_octomap
  PRIVATE
    octomap_vendor
    octomath_vendor
)
set_target_properties(octoplanner3d_dump_octomap PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(octoplanner3d_verify_path verify_path.cpp)
target_link_libraries(octoplanner3d_verify_path
  PRIVATE
    octoplanner3d_global_planner
)
set_target_properties(octoplanner3d_verify_path PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)

add_executable(octoplanner3d_make_test_octomap make_test_octomap.cpp)
target_link_libraries(octoplanner3d_make_test_octomap
  PRIVATE
    octomap_vendor
    octomath_vendor
)
set_target_properties(octoplanner3d_make_test_octomap PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)
if(BUILD_TESTING)
  add_executable(octoplanner3d_start_evidence_smoke start_evidence_smoke.cpp)
  target_link_libraries(octoplanner3d_start_evidence_smoke PRIVATE octoplanner3d_runtime octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_start_evidence_smoke COMMAND octoplanner3d_start_evidence_smoke)
  add_executable(octoplanner3d_surface_smoke surface_smoke.cpp)
  target_link_libraries(octoplanner3d_surface_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_surface_smoke COMMAND octoplanner3d_surface_smoke)
  add_executable(octoplanner3d_prepared_surface_smoke prepared_surface_smoke.cpp)
  target_link_libraries(octoplanner3d_prepared_surface_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_prepared_surface_smoke COMMAND octoplanner3d_prepared_surface_smoke)
  add_executable(octoplanner3d_surface_graph_smoke surface_graph_smoke.cpp surface_graph.cpp)
  target_link_libraries(octoplanner3d_surface_graph_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_surface_graph_smoke COMMAND octoplanner3d_surface_graph_smoke)
  add_executable(octoplanner3d_mls_backend_smoke mls_backend_smoke.cpp)
  target_link_libraries(octoplanner3d_mls_backend_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_mls_backend_smoke COMMAND octoplanner3d_mls_backend_smoke)
  add_executable(octoplanner3d_route_smoothing_smoke route_smoothing_smoke.cpp)
  target_link_libraries(octoplanner3d_route_smoothing_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_route_smoothing_smoke COMMAND octoplanner3d_route_smoothing_smoke)
  add_executable(test_body_geometry "${LINGTU_NAV_CPP_DIR}/../../../tests/nav/cpp/test_body_geometry.cpp")
  target_link_libraries(test_body_geometry PRIVATE lingtu_nav_body)
  add_test(NAME test_body_geometry COMMAND test_body_geometry)
  add_executable(octoplanner3d_xyz_smoke xyz_smoke.cpp)
  target_link_libraries(octoplanner3d_xyz_smoke PRIVATE octoplanner3d_runtime octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_xyz_smoke COMMAND octoplanner3d_xyz_smoke)
  add_executable(octoplanner3d_planning_slice_smoke planning_slice_smoke.cpp)
  target_link_libraries(octoplanner3d_planning_slice_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_planning_slice_smoke COMMAND octoplanner3d_planning_slice_smoke)
  add_executable(octoplanner3d_neighborhood_smoke neighborhood_smoke.cpp)
  target_link_libraries(octoplanner3d_neighborhood_smoke
    PRIVATE octoplanner3d_runtime octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_neighborhood_smoke
    COMMAND octoplanner3d_neighborhood_smoke)

  add_executable(octoplanner3d_grid_smoke grid_smoke.cpp)
  target_link_libraries(octoplanner3d_grid_smoke PRIVATE octoplanner3d_global_planner)
  add_test(NAME octoplanner3d_grid_smoke COMMAND octoplanner3d_grid_smoke)

  add_test(NAME octoplanner3d_verify_path_self_test
    COMMAND octoplanner3d_verify_path --self-test)

  add_executable(octoplanner3d_queue_node_compare_smoke queue_node_compare_smoke.cpp)
  target_link_libraries(octoplanner3d_queue_node_compare_smoke PRIVATE lingtu_settings)
  target_include_directories(octoplanner3d_queue_node_compare_smoke
    PRIVATE
      "${LINGTU_NAV_CPP_DIR}/../../maps/include"
      "${OCTOPLANNER3D_SOURCE_DIR}/planner/include"
      "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
      "${OCTOMAP_INCLUDE_DIR}"
  )
  add_test(
    NAME octoplanner3d_queue_node_compare_smoke
    COMMAND octoplanner3d_queue_node_compare_smoke)

  add_test(
    NAME octoplanner3d_make_test_octomap_smoke
    COMMAND octoplanner3d_make_test_octomap
      --output "${CMAKE_CURRENT_BINARY_DIR}/octoplanner3d_test_gap.bt")

  add_test(
    NAME octoplanner3d_edit_octomap_binary_ot_smoke
    COMMAND "${CMAKE_COMMAND}"
      -DMAKE_TEST=$<TARGET_FILE:octoplanner3d_make_test_octomap>
      -DEDITOR=$<TARGET_FILE:octoplanner3d_edit_octomap>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/edit_octomap_smoke
      -P "${CMAKE_CURRENT_LIST_DIR}/edit_octomap_smoke.cmake")

  add_executable(octoplanner3d_no_air_climb_smoke no_air_climb_smoke.cpp)
  add_executable(octoplanner3d_no_go_smoke no_go_smoke.cpp)
  add_executable(octoplanner3d_prepared_cache_smoke prepared_cache_smoke.cpp)
  target_include_directories(octoplanner3d_prepared_cache_smoke PRIVATE "${LINGTU_NAV_CPP_DIR}/../../maps/include")
  target_link_libraries(octoplanner3d_prepared_cache_smoke PRIVATE octoplanner3d_runtime octomap_vendor octomath_vendor)
  add_test(NAME octoplanner3d_prepared_cache_smoke COMMAND octoplanner3d_prepared_cache_smoke)
  target_include_directories(octoplanner3d_no_go_smoke PRIVATE "${LINGTU_NAV_CPP_DIR}/../../maps/include")
  target_link_libraries(octoplanner3d_no_go_smoke PRIVATE octoplanner3d_runtime octomap_vendor octomath_vendor)
  add_test(NAME octoplanner3d_no_go_smoke COMMAND octoplanner3d_no_go_smoke)
  target_link_libraries(octoplanner3d_no_air_climb_smoke
    PRIVATE
      octoplanner3d_runtime
      octomap_vendor
      octomath_vendor
  )
  set_target_properties(octoplanner3d_no_air_climb_smoke PROPERTIES
    BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
    INSTALL_RPATH "$ORIGIN"
  )
  add_test(
    NAME octoplanner3d_no_air_climb_smoke
    COMMAND octoplanner3d_no_air_climb_smoke)
endif()

add_executable(octoplanner3d_pcd_to_octomap pcd_to_octomap.cpp)
target_sources(octoplanner3d_pcd_to_octomap PRIVATE
  "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/io/scans.cpp"
  "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/io/pcd.cpp")
target_include_directories(octoplanner3d_pcd_to_octomap PRIVATE
  "${LINGTU_NAV_CPP_DIR}/../../maps/include")
target_link_libraries(octoplanner3d_pcd_to_octomap
  PUBLIC "$<BUILD_INTERFACE:lingtu_settings>"
  PRIVATE
    octomap_vendor
    octomath_vendor
)
if(OCTOPLANNER3D_ENABLE_PCD)
  target_compile_definitions(octoplanner3d_pcd_to_octomap PRIVATE OCTOPLANNER3D_ENABLE_PCD=1)
  target_include_directories(octoplanner3d_pcd_to_octomap PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(octoplanner3d_pcd_to_octomap PRIVATE ${PCL_LIBRARIES})
endif()
set_target_properties(octoplanner3d_pcd_to_octomap PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)
install(TARGETS octoplanner3d_pcd_to_octomap
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)

if(BUILD_TESTING)
  add_executable(octoplanner3d_sampled_pcd_smoke sampled_pcd_smoke.cpp)
  target_link_libraries(octoplanner3d_sampled_pcd_smoke PRIVATE octomap_vendor octomath_vendor)
  if(OCTOPLANNER3D_ENABLE_PCD)
    target_compile_definitions(octoplanner3d_sampled_pcd_smoke PRIVATE OCTOPLANNER3D_TEST_DIRECT_PCD=1)
    target_link_libraries(octoplanner3d_sampled_pcd_smoke PRIVATE octoplanner3d_pcd_converter)
  endif()
  add_test(NAME octoplanner3d_sampled_pcd_smoke
    COMMAND octoplanner3d_sampled_pcd_smoke
      $<TARGET_FILE:octoplanner3d_pcd_to_octomap>
      "${CMAKE_CURRENT_BINARY_DIR}/sampled-pcd-fixture"
      $<BOOL:${OCTOPLANNER3D_ENABLE_PCD}>)
  if(OCTOPLANNER3D_ENABLE_PCD)
    add_executable(octoplanner3d_ascii_pcd_test_converter pcd_to_octomap.cpp)
    target_sources(octoplanner3d_ascii_pcd_test_converter PRIVATE
      "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/io/scans.cpp"
      "${LINGTU_NAV_CPP_DIR}/../../maps/cpp/io/pcd.cpp")
    target_include_directories(octoplanner3d_ascii_pcd_test_converter PRIVATE
      "${LINGTU_NAV_CPP_DIR}/../../maps/include")
    target_link_libraries(octoplanner3d_ascii_pcd_test_converter
      PUBLIC "$<BUILD_INTERFACE:lingtu_settings>"
      PRIVATE octomap_vendor octomath_vendor)
    add_test(NAME octoplanner3d_sampled_ascii_pcd_smoke
      COMMAND octoplanner3d_sampled_pcd_smoke
        $<TARGET_FILE:octoplanner3d_ascii_pcd_test_converter>
        "${CMAKE_CURRENT_BINARY_DIR}/sampled-ascii-pcd-fixture" 0)
  endif()
endif()
