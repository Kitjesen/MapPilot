cmake_minimum_required(VERSION 3.14)
project(nav_kernel LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LOCAL_PLANNER_CPP_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../services/plan/local_planner/cpp")
add_subdirectory(
  "${LOCAL_PLANNER_CPP_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/local_planner_cpp")

# MSVC: treat source files as UTF-8 to avoid C4819 warnings.
if(MSVC)
  add_compile_options(/utf-8)
endif()

# Performance flags for all non-MSVC platforms, including aarch64.
# Do not enable relaxed floating-point math globally: validation must preserve NaN/Inf checks.
if(NOT MSVC)
  add_compile_options(-funroll-loops)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported)
  if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# xsimd removed: profile on S100P showed SIMD had no benefit on ARM Cortex-A55
# for our workload sizes. Compiler auto-vectorization handles the hot paths.
include(FetchContent)

# OpenMP is optional and is used for terrain/scoring parallelization.
find_package(OpenMP QUIET)

add_library(${PROJECT_NAME} STATIC
  src/path_follower_core.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/services/plan/local_planner/cpp>)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

# Dual mode: ament_cmake for ROS 2, standalone for Windows/CI.
if(DEFINED ENV{AMENT_PREFIX_PATH})
  # ROS 2 mode for S100P / sunrise aarch64.
  find_package(ament_cmake REQUIRED)

  if(OpenMP_CXX_FOUND)
    ament_export_dependencies(OpenMP)
  endif()

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

    # Helper: add test with OpenMP.
    macro(nav_kernel_add_test target src)
      ament_add_gtest(${target} ${src})
      target_include_directories(${target} PRIVATE include)
      target_link_libraries(${target} ${PROJECT_NAME} local_planner_cpp)
      if(OpenMP_CXX_FOUND)
        target_link_libraries(${target} OpenMP::OpenMP_CXX)
      endif()
    endmacro()

    nav_kernel_add_test(test_path_follower_core tests/test_path_follower_core.cpp)
    nav_kernel_add_test(test_pct_adapter_core tests/test_pct_adapter_core.cpp)
    nav_kernel_add_test(test_param_sensitivity tests/test_param_sensitivity.cpp)
    nav_kernel_add_test(test_path_edge_cases tests/test_path_edge_cases.cpp)
    nav_kernel_add_test(test_benchmark tests/test_benchmark.cpp)
    nav_kernel_add_test(test_validation tests/test_validation.cpp)
    nav_kernel_add_test(test_map_layers_core tests/test_map_layers_core.cpp)
  endif()

  ament_package()

else()
  # Standalone mode for Windows, CI, and local development.

  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  FetchContent_Declare(taskflow
    GIT_REPOSITORY https://github.com/taskflow/taskflow.git
    GIT_TAG        v3.8.0)

  set(TF_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_CUDA OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_PROFILER OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_SYCL OFF CACHE BOOL "" FORCE)
  set(TF_BUILD_TESTS OFF CACHE BOOL "" FORCE)

  FetchContent_MakeAvailable(googletest taskflow)

  # OpenMP is optional and used for baseline comparison.
  find_package(OpenMP QUIET)

  enable_testing()
  include(GoogleTest)

  add_executable(test_path_follower_core tests/test_path_follower_core.cpp)
  target_include_directories(test_path_follower_core PRIVATE include)
  target_link_libraries(test_path_follower_core GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_path_follower_core)

  add_executable(test_pct_adapter_core tests/test_pct_adapter_core.cpp)
  target_include_directories(test_pct_adapter_core PRIVATE include)
  target_link_libraries(test_pct_adapter_core GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_pct_adapter_core)

  add_executable(test_param_sensitivity tests/test_param_sensitivity.cpp)
  target_include_directories(test_param_sensitivity PRIVATE include)
  target_link_libraries(test_param_sensitivity GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_param_sensitivity)

  add_executable(test_path_edge_cases tests/test_path_edge_cases.cpp)
  target_include_directories(test_path_edge_cases PRIVATE include)
  target_link_libraries(test_path_edge_cases GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_path_edge_cases)

  add_executable(test_benchmark tests/test_benchmark.cpp)
  target_include_directories(test_benchmark PRIVATE include)
  target_link_libraries(test_benchmark GTest::gtest_main Taskflow ${PROJECT_NAME} local_planner_cpp)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_benchmark OpenMP::OpenMP_CXX)
  endif()
  gtest_discover_tests(test_benchmark)

  add_executable(test_validation tests/test_validation.cpp)
  target_include_directories(test_validation PRIVATE include)
  target_link_libraries(test_validation GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_validation)

  add_executable(test_map_layers_core tests/test_map_layers_core.cpp)
  target_include_directories(test_map_layers_core PRIVATE include)
  target_link_libraries(test_map_layers_core GTest::gtest_main ${PROJECT_NAME} local_planner_cpp)
  gtest_discover_tests(test_map_layers_core)

  option(NAV_KERNEL_BUILD_PYTHON_BINDINGS "Build nav_kernel Python bindings" OFF)
  if(NAV_KERNEL_BUILD_PYTHON_BINDINGS)
    find_package(Python COMPONENTS Interpreter Development QUIET)
  endif()
  if(NAV_KERNEL_BUILD_PYTHON_BINDINGS AND Python_FOUND AND EXISTS "${Python_INCLUDE_DIRS}/Python.h")
    # nanobind replaces pybind11 for faster builds, smaller modules, and numpy zero-copy support.
    FetchContent_Declare(nanobind
      GIT_REPOSITORY https://github.com/wjakob/nanobind.git
      GIT_TAG        v2.12.0)
    FetchContent_MakeAvailable(nanobind)
    set(NAV_KERNEL_BINDING_SOURCES
      bindings/bindings.cpp
      bindings/bind_types.cpp
      bindings/bind_map_layers.cpp
      bindings/bind_path_follower.cpp
      bindings/bind_waypoint_helpers.cpp
      bindings/bind_local_planner.cpp
      bindings/bind_terrain.cpp)
    nanobind_add_module(lingtu_nav_kernel ${NAV_KERNEL_BINDING_SOURCES})
    target_include_directories(lingtu_nav_kernel PRIVATE include)
    target_link_libraries(lingtu_nav_kernel PRIVATE ${PROJECT_NAME} local_planner_cpp)
  elseif(NAV_KERNEL_BUILD_PYTHON_BINDINGS)
    message(WARNING "Skipping nav_kernel Python bindings because Python development headers were not found")
  endif()
endif()
