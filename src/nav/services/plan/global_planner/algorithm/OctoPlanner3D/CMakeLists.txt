cmake_minimum_required(VERSION 3.16)
project(GlobalPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS io common)

add_library(octomap SHARED IMPORTED GLOBAL)
set_target_properties(octomap PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomap.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/octomap/include"
)

add_library(octomath SHARED IMPORTED GLOBAL)
set_target_properties(octomath PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomath.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/octomap/include"
)

add_subdirectory(octomap)
add_subdirectory(planner)

add_executable(globalPlan main.cpp)
target_link_libraries(globalPlan PRIVATE GlobalPlanner Octomap)
