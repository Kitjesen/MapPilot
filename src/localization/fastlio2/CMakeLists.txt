cmake_minimum_required(VERSION 3.16)
project(fastlio2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(FASTLIO2_ENABLE_INTERNAL_TEST_HOOKS
  "Expose Fast-LIO2 internal test hooks from fastlio2_core" OFF)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

set(FASTLIO2_CORE_SOURCES
  src/map_builder/commons.cpp
  src/map_builder/ieskf.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/ikd_Tree.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
)

add_library(fastlio2_core STATIC ${FASTLIO2_CORE_SOURCES})
target_compile_features(fastlio2_core PUBLIC cxx_std_17)
target_include_directories(fastlio2_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/map_builder
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio2_core PUBLIC Eigen3::Eigen Threads::Threads ${PCL_LIBRARIES})
set_target_properties(fastlio2_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(FASTLIO2_ENABLE_INTERNAL_TEST_HOOKS)
  target_compile_definitions(fastlio2_core PUBLIC LINGTU_IKD_TREE_TESTING=1)
endif()

if(OpenMP_CXX_FOUND)
  target_compile_definitions(fastlio2_core PUBLIC MP_EN MP_PROC_NUM=4)
  target_link_libraries(fastlio2_core PUBLIC OpenMP::OpenMP_CXX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(fastlio2_core PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()
