cmake_minimum_required(VERSION 3.13.0)
project(planning_core CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(PCT_CPU_ARCH_FLAGS "" CACHE STRING "Optional CPU instruction baseline for PCT native modules")
if(PCT_CPU_ARCH_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PCT_CPU_ARCH_FLAGS}")
endif()
option(
  LINGTU_PCT_BUILD_NATIVE_MODULES
  "Build legacy PCT C++/pybind native modules"
  OFF
)
option(
  LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE
  "Build legacy GTSAM-backed PCT native comparison modules"
  OFF
)
if(LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE AND NOT LINGTU_PCT_BUILD_NATIVE_MODULES)
  set(LINGTU_PCT_BUILD_NATIVE_MODULES ON)
endif()

# set(GTSAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/gtsam-4.1.1/install/lib/cmake/GTSAM)

if(LINGTU_PCT_BUILD_NATIVE_MODULES)
  find_package(Eigen3 REQUIRED)
  include_directories(
    src
    ${EIGEN3_INCLUDE_DIR}
  )

  add_subdirectory(src/common)
  add_subdirectory(3rdparty/pybind11)
  add_subdirectory(src/a_star)
  add_subdirectory(src/map_manager)

  if(LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE)
    find_package(GTSAM REQUIRED)
    include_directories(
      ${GTSAM_INCLUDE_DIR}
      ${GTSAM_INCLUDE_DIRS}
    )
    add_subdirectory(src/trajectory_optimization)
    add_subdirectory(src/ele_planner)
  else()
    message(STATUS "Skipping legacy PCT GTSAM native modules; runtime uses Rust gpmp_trajectory_optimizer by default")
  endif()
else()
  message(STATUS "Skipping PCT C++ native modules; runtime uses Rust gpmp_trajectory_optimizer by default")
endif()
