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

add_library(octoplanner3d_global_planner STATIC
  "${OCTOPLANNER3D_SOURCE_DIR}/planner/src/global_planner.cpp"
)
target_include_directories(octoplanner3d_global_planner
  PUBLIC
    "${OCTOPLANNER3D_SOURCE_DIR}/planner/include"
    "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
)
target_link_libraries(octoplanner3d_global_planner
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

add_library(octoplanner3d_runtime STATIC
  octoplanner3d_core.cpp
)
target_include_directories(octoplanner3d_runtime
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${LINGTU_NAV_CPP_DIR}"
)
target_link_libraries(octoplanner3d_runtime
  PRIVATE
    octoplanner3d_global_planner
)
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
  add_executable(octoplanner3d_queue_node_compare_smoke queue_node_compare_smoke.cpp)
  target_include_directories(octoplanner3d_queue_node_compare_smoke
    PRIVATE
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
target_link_libraries(octoplanner3d_pcd_to_octomap
  PRIVATE
    octomap_vendor
    octomath_vendor
)
if(OCTOPLANNER3D_ENABLE_PCD)
  target_compile_definitions(octoplanner3d_pcd_to_octomap PRIVATE OCTOPLANNER3D_ENABLE_PCD=1)
  target_link_libraries(octoplanner3d_pcd_to_octomap PRIVATE octoplanner3d_pcd_converter)
endif()
set_target_properties(octoplanner3d_pcd_to_octomap PROPERTIES
  BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN"
)
install(TARGETS octoplanner3d_pcd_to_octomap
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT lingtu_runtime)
