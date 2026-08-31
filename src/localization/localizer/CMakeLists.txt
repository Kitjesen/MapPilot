cmake_minimum_required(VERSION 3.16)
project(localizer LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenMP QUIET)

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

get_filename_component(LINGTU_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
if(NOT CPU_BBS3D_ROOT)
  set(_DEFAULT_BBS3D_ROOT "${LINGTU_REPO_ROOT}/third_party/3d_bbs/install")
  if(EXISTS "${_DEFAULT_BBS3D_ROOT}")
    set(CPU_BBS3D_ROOT "${_DEFAULT_BBS3D_ROOT}" CACHE PATH
      "Optional local install prefix for cpu_bbs3d" FORCE)
  endif()
endif()
if(NOT SMALL_GICP_ROOT)
  set(_DEFAULT_SMALL_GICP_ROOT
    "${LINGTU_REPO_ROOT}/third_party/research_localization/small_gicp")
  if(EXISTS "${_DEFAULT_SMALL_GICP_ROOT}/include/small_gicp")
    set(SMALL_GICP_ROOT "${_DEFAULT_SMALL_GICP_ROOT}" CACHE PATH
      "Optional small_gicp source or install prefix" FORCE)
  endif()
endif()

find_library(CPU_BBS3D_LIBRARY
  NAMES cpu_bbs3d libcpu_bbs3d.so
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local)
find_path(CPU_BBS3D_INCLUDE_DIR
  NAMES cpu_bbs3d/bbs3d.hpp
  HINTS ${CPU_BBS3D_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local /usr/local/include /usr/local/include/bbs3d)

if(NOT CPU_BBS3D_LIBRARY OR NOT CPU_BBS3D_INCLUDE_DIR)
  if(LINGTU_REQUIRE_BBS3D)
    message(FATAL_ERROR
      "LINGTU_REQUIRE_BBS3D=ON but cpu_bbs3d was not found. "
      "Set CPU_BBS3D_ROOT to its install prefix.")
  endif()
  set(CPU_BBS3D_LIBRARY "")
  set(CPU_BBS3D_INCLUDE_DIR "")
  message(WARNING "BBS3D not found; localizer will use seeded ICP only")
else()
  message(STATUS "BBS3D enabled: ${CPU_BBS3D_LIBRARY}")
endif()

find_path(SMALL_GICP_INCLUDE_DIR
  NAMES small_gicp/pcl/pcl_registration.hpp
  HINTS ${SMALL_GICP_ROOT}
  PATH_SUFFIXES include
  PATHS /usr/local)
if(NOT LINGTU_ENABLE_SMALL_GICP OR NOT SMALL_GICP_INCLUDE_DIR OR NOT OpenMP_CXX_FOUND)
  set(SMALL_GICP_INCLUDE_DIR "")
  message(STATUS "small_gicp disabled or unavailable; localizer will use PCL GICP")
else()
  message(STATUS "small_gicp enabled: ${SMALL_GICP_INCLUDE_DIR}")
endif()

add_library(localizer STATIC
  src/localizers/commons.cpp
  src/localizers/icp_localizer.cpp
  src/localizers/bbs3d_global_localizer.cpp)
target_compile_features(localizer PUBLIC cxx_std_17)
target_include_directories(localizer
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/localizers"
    ${PCL_INCLUDE_DIRS}
    ${CPU_BBS3D_INCLUDE_DIR}
    ${SMALL_GICP_INCLUDE_DIR})
target_link_libraries(localizer
  PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${CPU_BBS3D_LIBRARY})

if(CPU_BBS3D_LIBRARY AND CPU_BBS3D_INCLUDE_DIR)
  target_compile_definitions(localizer PUBLIC LINGTU_ENABLE_BBS3D=1)
endif()
if(SMALL_GICP_INCLUDE_DIR)
  target_compile_definitions(localizer PUBLIC LINGTU_ENABLE_SMALL_GICP=1)
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(localizer PUBLIC OpenMP::OpenMP_CXX)
endif()

if(MSVC)
  target_compile_options(localizer PRIVATE /W4)
else()
  target_compile_options(localizer PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(localizer_cli src/localizer_cli.cpp)
target_link_libraries(localizer_cli PRIVATE localizer)
