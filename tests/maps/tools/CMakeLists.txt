cmake_minimum_required(VERSION 3.16)

project(lingtu_maps_evidence_tool LANGUAGES CXX)

include(CTest)

find_path(OCTOMAP_INCLUDE_DIR NAMES octomap/OcTree.h REQUIRED)
find_library(OCTOMAP_LIBRARY NAMES octomap REQUIRED)
find_library(OCTOMATH_LIBRARY NAMES octomath REQUIRED)

set(LINGTU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

add_executable(
  lingtu_maps_evidence_tool
  evidence_tool.cpp
  "${LINGTU_ROOT}/src/maps/cpp/build/pcd.cpp"
  "${LINGTU_ROOT}/src/maps/cpp/build/saved_scans.cpp"
  "${LINGTU_ROOT}/src/maps/cpp/json.cpp"
)
target_compile_features(lingtu_maps_evidence_tool PRIVATE cxx_std_17)
target_include_directories(
  lingtu_maps_evidence_tool
  PRIVATE
    "${LINGTU_ROOT}/src/maps/include"
    "${OCTOMAP_INCLUDE_DIR}"
)
target_link_libraries(
  lingtu_maps_evidence_tool
  PRIVATE
    "${OCTOMAP_LIBRARY}"
    "${OCTOMATH_LIBRARY}"
)
if(MSVC)
  target_compile_options(lingtu_maps_evidence_tool PRIVATE /W4 /FS)
else()
  target_compile_options(lingtu_maps_evidence_tool PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME lingtu_maps_evidence_tool_self_test
         COMMAND lingtu_maps_evidence_tool self-test)
