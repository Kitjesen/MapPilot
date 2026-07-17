cmake_minimum_required(VERSION 3.16)
project(lingtu_pointcloud_codec LANGUAGES CXX)

add_library(lingtu_pointcloud_codec SHARED
  src/pointcloud_codec.cpp
)

target_compile_features(lingtu_pointcloud_codec PRIVATE cxx_std_17)
target_include_directories(lingtu_pointcloud_codec
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(MSVC)
  target_compile_options(lingtu_pointcloud_codec PRIVATE /O2 /W4)
else()
  target_compile_options(lingtu_pointcloud_codec PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()
