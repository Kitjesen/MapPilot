cmake_minimum_required(VERSION 3.14)
project(nav_core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: 源文件为 UTF-8, 消除 C4819 + 防止中文字符串乱码
if(MSVC)
  add_compile_options(/utf-8)
endif()

# ── 双模式: ament_cmake (ROS2) 或 standalone (Windows/CI) ──
if(DEFINED ENV{AMENT_PREFIX_PATH})
  # ── ROS2 模式 ──
  find_package(ament_cmake REQUIRED)

  # header-only library
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

  # gtest
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_path_follower_core test/test_path_follower_core.cpp)
    target_include_directories(test_path_follower_core PRIVATE include)
    ament_add_gtest(test_pct_adapter_core test/test_pct_adapter_core.cpp)
    target_include_directories(test_pct_adapter_core PRIVATE include)
    ament_add_gtest(test_local_planner_core test/test_local_planner_core.cpp)
    target_include_directories(test_local_planner_core PRIVATE include)
    ament_add_gtest(test_param_sensitivity test/test_param_sensitivity.cpp)
    target_include_directories(test_param_sensitivity PRIVATE include)
    ament_add_gtest(test_path_edge_cases test/test_path_edge_cases.cpp)
    target_include_directories(test_path_edge_cases PRIVATE include)
    ament_add_gtest(test_benchmark test/test_benchmark.cpp)
    target_include_directories(test_benchmark PRIVATE include)
    ament_add_gtest(test_validation test/test_validation.cpp)
    target_include_directories(test_validation PRIVATE include)
  endif()

  ament_package()

else()
  # ── Standalone 模式 (Windows / CI) ──
  include(FetchContent)

  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)

  add_executable(test_path_follower_core test/test_path_follower_core.cpp)
  target_include_directories(test_path_follower_core PRIVATE include)
  target_link_libraries(test_path_follower_core GTest::gtest_main)
  gtest_discover_tests(test_path_follower_core)

  add_executable(test_pct_adapter_core test/test_pct_adapter_core.cpp)
  target_include_directories(test_pct_adapter_core PRIVATE include)
  target_link_libraries(test_pct_adapter_core GTest::gtest_main)
  gtest_discover_tests(test_pct_adapter_core)

  add_executable(test_local_planner_core test/test_local_planner_core.cpp)
  target_include_directories(test_local_planner_core PRIVATE include)
  target_link_libraries(test_local_planner_core GTest::gtest_main)
  gtest_discover_tests(test_local_planner_core)

  add_executable(test_param_sensitivity test/test_param_sensitivity.cpp)
  target_include_directories(test_param_sensitivity PRIVATE include)
  target_link_libraries(test_param_sensitivity GTest::gtest_main)
  gtest_discover_tests(test_param_sensitivity)

  add_executable(test_path_edge_cases test/test_path_edge_cases.cpp)
  target_include_directories(test_path_edge_cases PRIVATE include)
  target_link_libraries(test_path_edge_cases GTest::gtest_main)
  gtest_discover_tests(test_path_edge_cases)

  add_executable(test_benchmark test/test_benchmark.cpp)
  target_include_directories(test_benchmark PRIVATE include)
  target_link_libraries(test_benchmark GTest::gtest_main)
  gtest_discover_tests(test_benchmark)

  add_executable(test_validation test/test_validation.cpp)
  target_include_directories(test_validation PRIVATE include)
  target_link_libraries(test_validation GTest::gtest_main)
  gtest_discover_tests(test_validation)

  # pybind11 (可选)
  find_package(Python3 COMPONENTS Interpreter Development QUIET)
  if(Python3_FOUND)
    FetchContent_Declare(pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11.git
      GIT_TAG        v2.12.0)
    FetchContent_MakeAvailable(pybind11)
    pybind11_add_module(_nav_core bindings/py_nav_core.cpp)
    target_include_directories(_nav_core PRIVATE include)
  endif()
endif()
