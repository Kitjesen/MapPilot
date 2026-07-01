cmake_minimum_required(VERSION 3.16)
project(octoplanner3d_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(OCTOPLANNER3D_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "Path to the local OctoPlanner3D source")

if(NOT EXISTS "${OCTOPLANNER3D_SOURCE_DIR}/planner/include/global_planner.h")
  message(FATAL_ERROR "OCTOPLANNER3D_SOURCE_DIR must point to a checkout with planner/include/global_planner.h: ${OCTOPLANNER3D_SOURCE_DIR}")
endif()

find_package(PCL QUIET COMPONENTS common io octree)
set(OCTOPLANNER3D_PCD_CONVERTER_HEADER "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include/pcd2octomap_converter.h")
set(OCTOPLANNER3D_PCD_CONVERTER_SOURCE "${OCTOPLANNER3D_SOURCE_DIR}/octomap/src/pcd2octomap_converter.cpp")
set(OCTOPLANNER3D_ENABLE_PCD OFF)
if(PCL_FOUND)
  if(EXISTS "${OCTOPLANNER3D_PCD_CONVERTER_HEADER}" AND EXISTS "${OCTOPLANNER3D_PCD_CONVERTER_SOURCE}")
    set(OCTOPLANNER3D_ENABLE_PCD ON)
    message(STATUS "PCL found: enabling .pcd -> OctoMap conversion in octoplanner3d_headless")
  else()
    message(WARNING "PCL found but OctoPlanner3D PCD converter source/header missing: building octoplanner3d_headless with .bt OctoMap input support only")
  endif()
else()
  message(WARNING "PCL not found: building octoplanner3d_headless with .bt OctoMap input support only")
endif()

add_library(octomap_vendor SHARED IMPORTED GLOBAL)
set_target_properties(octomap_vendor PROPERTIES
  IMPORTED_LOCATION "${OCTOPLANNER3D_SOURCE_DIR}/lib/liboctomap.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
)

add_library(octomath_vendor SHARED IMPORTED GLOBAL)
set_target_properties(octomath_vendor PROPERTIES
  IMPORTED_LOCATION "${OCTOPLANNER3D_SOURCE_DIR}/lib/liboctomath.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${OCTOPLANNER3D_SOURCE_DIR}/octomap/include"
)

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
)
target_link_libraries(octoplanner3d_runtime
  PRIVATE
    octoplanner3d_global_planner
)
if(OCTOPLANNER3D_ENABLE_PCD)
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

option(OCTOPLANNER3D_BUILD_PYTHON_BINDINGS "Build the ROS2-free OctoPlanner3D Python extension" OFF)
if(OCTOPLANNER3D_BUILD_PYTHON_BINDINGS)
  find_package(Python COMPONENTS Interpreter Development REQUIRED)
endif()
if(OCTOPLANNER3D_BUILD_PYTHON_BINDINGS)
  add_library(_native MODULE py_octoplanner3d.cpp)
  target_link_libraries(_native PRIVATE octoplanner3d_runtime Python::Python)
  set_target_properties(_native PROPERTIES
    PREFIX ""
    BUILD_RPATH "${OCTOPLANNER3D_SOURCE_DIR}/lib"
    INSTALL_RPATH "$ORIGIN"
  )
  if(Python_SOABI)
    set_target_properties(_native PROPERTIES SUFFIX ".${Python_SOABI}.so")
  endif()
endif()

install(TARGETS octoplanner3d_headless RUNTIME DESTINATION bin)
