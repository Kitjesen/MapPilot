cmake_minimum_required(VERSION 3.8)
project(pct_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(FILES
  launch/mapping_launch.py
  launch/system_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(FILES launch/test/planner_only_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
  RENAME planner_only_launch.py
)
install(FILES launch/test/test_planning_launch.py
  DESTINATION share/${PROJECT_NAME}/launch/test
)

# Install Planner Code (as data to be imported by scripts)
install(DIRECTORY planner
  DESTINATION share/${PROJECT_NAME}
)

# Install Resources (Map files etc)
install(DIRECTORY rsc
  DESTINATION share/${PROJECT_NAME}
)
# tomography/config 供 global_planner 发布 tomogram 时加载 prototype.py（路径 share/pct_planner/tomography/config/）
install(DIRECTORY tomography
  DESTINATION share/${PROJECT_NAME}/tomography
)

install(PROGRAMS
  planner/scripts/global_planner.py
  planner/scripts/pct_planner_astar.py
  planner/scripts/fake_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
