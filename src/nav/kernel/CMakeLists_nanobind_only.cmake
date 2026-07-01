# Minimal CMakeLists for building lingtu_nav_kernel.so with nanobind
# Usage on S100P:
#   cp CMakeLists_nanobind_only.cmake CMakeLists.txt
#   cmake -B build_nb -DCMAKE_BUILD_TYPE=Release
#   cmake --build build_nb -j4
#   cp build_nb/lingtu_nav_kernel*.so ~/data/SLAM/navigation/src/
cmake_minimum_required(VERSION 3.14)
project(nav_kernel_binding LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LOCAL_PLANNER_CPP_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../services/plan/local_planner/cpp")
add_subdirectory(
  "${LOCAL_PLANNER_CPP_DIR}"
  "${CMAKE_CURRENT_BINARY_DIR}/local_planner_cpp")

# nanobind requires find_package(Python ...) not find_package(Python3 ...)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# nanobind installed via pip: python3 -c "import nanobind; print(nanobind.cmake_dir())"
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NB_RET)
if(NOT NB_RET EQUAL 0)
  message(FATAL_ERROR "nanobind not found. Install: pip install nanobind")
endif()
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Build the Python extension module
nanobind_add_module(lingtu_nav_kernel bindings/bindings.cpp)
target_include_directories(lingtu_nav_kernel PRIVATE include)
target_link_libraries(lingtu_nav_kernel PRIVATE local_planner_cpp)
