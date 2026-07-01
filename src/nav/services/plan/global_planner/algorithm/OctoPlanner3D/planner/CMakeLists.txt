add_library(GlobalPlanner STATIC src/global_planner.cpp)

target_include_directories(GlobalPlanner
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(GlobalPlanner
  PUBLIC
    octomap
    octomath
)
