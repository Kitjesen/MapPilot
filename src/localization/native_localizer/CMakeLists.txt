cmake_minimum_required(VERSION 3.16)
project(lingtu_native_localizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(OpenMP)

option(LINGTU_REQUIRE_BBS3D "Fail configure if cpu_bbs3d is not available" OFF)
set(CPU_BBS3D_ROOT "" CACHE PATH "Optional local install prefix for cpu_bbs3d")
if(NOT CPU_BBS3D_ROOT AND DEFINED ENV{CPU_BBS3D_ROOT})
  set(CPU_BBS3D_ROOT "$ENV{CPU_BBS3D_ROOT}")
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
if(OpenMP_CXX_FOUND)
  target_link_libraries(lingtu_localizer_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(localizer_cli localizer_cli.cpp)
target_link_libraries(localizer_cli PRIVATE lingtu_localizer_core)
