cmake_minimum_required(VERSION 3.16)
project(lingtu_native_localizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenMP)

option(LINGTU_REQUIRE_BBS3D "Fail configure if cpu_bbs3d is not available" OFF)
option(LINGTU_ENABLE_SMALL_GICP "Use small_gicp for relocalization refine when available" ON)
set(CPU_BBS3D_ROOT "" CACHE PATH "Optional local install prefix for cpu_bbs3d")
set(SMALL_GICP_ROOT "" CACHE PATH "Optional small_gicp source or install prefix")
if(NOT CPU_BBS3D_ROOT AND DEFINED ENV{CPU_BBS3D_ROOT})
  set(CPU_BBS3D_ROOT "$ENV{CPU_BBS3D_ROOT}")
endif()
if(NOT SMALL_GICP_ROOT AND DEFINED ENV{SMALL_GICP_ROOT})
  set(SMALL_GICP_ROOT "$ENV{SMALL_GICP_ROOT}")
endif()
if(NOT CPU_BBS3D_ROOT)
  get_filename_component(_LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(_LINGTU_DEFAULT_BBS3D_ROOT "${_LINGTU_REPO_ROOT}/third_party/3d_bbs/install")
  if(EXISTS "${_LINGTU_DEFAULT_BBS3D_ROOT}")
    set(CPU_BBS3D_ROOT "${_LINGTU_DEFAULT_BBS3D_ROOT}" CACHE PATH
      "Optional local install prefix for cpu_bbs3d" FORCE)
  endif()
endif()
if(NOT SMALL_GICP_ROOT)
  get_filename_component(_LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(_LINGTU_DEFAULT_SMALL_GICP_ROOT
    "${_LINGTU_REPO_ROOT}/third_party/research_localization/small_gicp")
  if(EXISTS "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}/include/small_gicp")
    set(SMALL_GICP_ROOT "${_LINGTU_DEFAULT_SMALL_GICP_ROOT}" CACHE PATH
      "Optional small_gicp source or install prefix" FORCE)
  endif()
endif()

find_library(CPU_BBS3D_LIBRARY
  NAMES cpu_bbs3d libcpu_bbs3d.so
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local
)
find_path(CPU_BBS3D_INCLUDE_DIR
  NAMES cpu_bbs3d/bbs3d.hpp
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local /usr/local/include /usr/local/include/bbs3d
)

if(CPU_BBS3D_LIBRARY AND CPU_BBS3D_INCLUDE_DIR)
  set(LINGTU_NATIVE_BBS3D_LINK ${CPU_BBS3D_LIBRARY})
  set(LINGTU_NATIVE_BBS3D_INCLUDE ${CPU_BBS3D_INCLUDE_DIR})
else()
  if(LINGTU_REQUIRE_BBS3D)
    message(FATAL_ERROR
      "LINGTU_REQUIRE_BBS3D=ON but cpu_bbs3d was not found. "
      "Set CPU_BBS3D_ROOT to its install prefix.")
  endif()
  set(LINGTU_NATIVE_BBS3D_LINK "")
  set(LINGTU_NATIVE_BBS3D_INCLUDE "")
endif()
find_path(SMALL_GICP_INCLUDE_DIR
  NAMES small_gicp/pcl/pcl_registration.hpp
  HINTS ${SMALL_GICP_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local)
if(NOT LINGTU_ENABLE_SMALL_GICP OR NOT SMALL_GICP_INCLUDE_DIR OR NOT OpenMP_CXX_FOUND)
  set(SMALL_GICP_INCLUDE_DIR "")
endif()

add_library(lingtu_localizer_core
  ../localizer/src/localizers/commons.cpp
  ../localizer/src/localizers/icp_localizer.cpp
  ../localizer/src/localizers/bbs3d_global_localizer.cpp
)
target_include_directories(lingtu_localizer_core
  PUBLIC
    ../localizer/src/localizers
    ${PCL_INCLUDE_DIRS}
    ${LINGTU_NATIVE_BBS3D_INCLUDE}
    ${SMALL_GICP_INCLUDE_DIR}
)
target_link_libraries(lingtu_localizer_core
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${LINGTU_NATIVE_BBS3D_LINK}
)
if(CPU_BBS3D_LIBRARY AND CPU_BBS3D_INCLUDE_DIR)
  target_compile_definitions(lingtu_localizer_core PUBLIC LINGTU_ENABLE_BBS3D=1)
endif()
if(SMALL_GICP_INCLUDE_DIR)
  target_compile_definitions(lingtu_localizer_core PUBLIC LINGTU_ENABLE_SMALL_GICP=1)
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(lingtu_localizer_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(localizer_cli localizer_cli.cpp)
target_link_libraries(localizer_cli PRIVATE lingtu_localizer_core)
