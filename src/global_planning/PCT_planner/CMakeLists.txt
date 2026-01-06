cmake_minimum_required(VERSION 3.8)
project(pct_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Planner Code (as data to be imported by scripts)
install(DIRECTORY planner
  DESTINATION share/${PROJECT_NAME}
)

# Install Entry Point Script
install(PROGRAMS planner/scripts/global_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
