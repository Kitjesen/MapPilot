cmake_minimum_required(VERSION 3.20)
project(lingtu_ufomap_miss_regression LANGUAGES CXX)
include(CTest)
get_filename_component(LINGTU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
set(UFOMAP_SOURCE_DIR "${LINGTU_ROOT}/third_party/research_nav/dufomap/ufomap"
    CACHE PATH "Pinned UFOMap source checkout")
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(TBB REQUIRED)
find_path(UFOMAP_LZF_INCLUDE_DIR liblzf/lzf.h REQUIRED)
find_library(UFOMAP_LZ4_LIBRARY NAMES lz4 REQUIRED)

# Compile the real map I/O support, needed by the actual Map vtable. This
# standalone test does not import the upstream project or any production target.
add_library(ufomap_regression_io STATIC "${UFOMAP_SOURCE_DIR}/src/map/io.cpp")
target_compile_features(ufomap_regression_io PUBLIC cxx_std_20)
target_include_directories(ufomap_regression_io PUBLIC
  "${UFOMAP_SOURCE_DIR}/include" "${UFOMAP_LZF_INCLUDE_DIR}")
target_link_libraries(ufomap_regression_io PUBLIC "${UFOMAP_LZ4_LIBRARY}" Threads::Threads)

add_executable(ufomap_miss_serial_test occupancy_miss_test.cpp)
target_link_libraries(ufomap_miss_serial_test PRIVATE ufomap_regression_io)
add_test(NAME ufomap_occupancy_miss_serial COMMAND ufomap_miss_serial_test serial)

add_executable(ufomap_miss_parallel_test occupancy_miss_test.cpp)
target_compile_definitions(ufomap_miss_parallel_test PRIVATE UFO_PARALLEL=1)
target_link_libraries(ufomap_miss_parallel_test PRIVATE
  ufomap_regression_io TBB::tbb OpenMP::OpenMP_CXX)
add_test(NAME ufomap_occupancy_miss_parallel_build_serial COMMAND ufomap_miss_parallel_test serial)
add_test(NAME ufomap_occupancy_miss_parallel_build_parallel COMMAND ufomap_miss_parallel_test parallel)
