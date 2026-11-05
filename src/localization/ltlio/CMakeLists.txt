cmake_minimum_required(VERSION 3.16)
project(ltlio LANGUAGES CXX)

include("${CMAKE_CURRENT_LIST_DIR}/../../../scripts/build/cmake/settings.cmake")

option(LTLIO_ENABLE_INTERNAL_TEST_HOOKS "Build LTLIO tree lifecycle regressions" OFF)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(Threads REQUIRED)
find_package(OpenMP QUIET)

add_library(ltlio_core STATIC
  frontend.cpp
  detail/IMUProcess.cpp
  detail/ESEKF.cpp
  detail/ElevatorProcess.cpp
  detail/ElevatorSelfExit.cpp
  detail/LidarPipeline.cpp
  detail/ikd_Tree.cpp
)
target_compile_features(ltlio_core PUBLIC cxx_std_17)
target_include_directories(ltlio_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${PCL_INCLUDE_DIRS})
target_link_libraries(ltlio_core PUBLIC "$<BUILD_INTERFACE:lingtu_settings>" Eigen3::Eigen Threads::Threads ${PCL_LIBRARIES})
set_target_properties(ltlio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
  # PCL points require correctly aligned STL stable_sort temporary storage.
  target_compile_definitions(ltlio_core PUBLIC _ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()
if(OpenMP_CXX_FOUND)
  target_compile_definitions(ltlio_core PRIVATE MP_EN MP_PROC_NUM=3)
  target_link_libraries(ltlio_core PUBLIC OpenMP::OpenMP_CXX)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Keep the pinned numerical reference's optimization and bounded Eigen build memory.
  target_compile_options(ltlio_core PRIVATE -O1 -Wall -Wextra -Wpedantic)
endif()

if(LTLIO_ENABLE_INTERNAL_TEST_HOOKS)
  # The instrumented tree is a separate test executable, never linked with
  # a differently defined production tree instantiation.
  add_executable(test_ikd_tree_concurrency
    ../../../tests/localization/cpp/slam/test_ikd_tree_concurrency.cpp
    detail/ikd_Tree.cpp)
  target_compile_features(test_ikd_tree_concurrency PRIVATE cxx_std_17)
  target_compile_definitions(test_ikd_tree_concurrency PRIVATE LINGTU_IKD_TREE_TESTING=1)
  target_include_directories(test_ikd_tree_concurrency PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/.. ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ikd_tree_concurrency PRIVATE Eigen3::Eigen Threads::Threads ${PCL_LIBRARIES})
endif()
