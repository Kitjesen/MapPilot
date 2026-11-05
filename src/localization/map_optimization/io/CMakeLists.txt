cmake_minimum_required(VERSION 3.20)
project(lingtu_mapping_io LANGUAGES CXX)

get_filename_component(LINGTU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
add_library(lingtu_mapping_io STATIC ../cloud_io.cpp)
target_compile_features(lingtu_mapping_io PUBLIC cxx_std_17)
target_include_directories(lingtu_mapping_io PUBLIC "${LINGTU_ROOT}/src")

include(CTest)
if(BUILD_TESTING)
  add_executable(mapping_io_test
    "${LINGTU_ROOT}/tests/localization/cpp/map_optimization/cloud_io_test.cpp")
  target_link_libraries(mapping_io_test PRIVATE lingtu_mapping_io)
  add_test(NAME mapping_io COMMAND mapping_io_test)
endif()
