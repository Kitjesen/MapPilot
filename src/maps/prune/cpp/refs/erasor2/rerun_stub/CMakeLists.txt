cmake_minimum_required(VERSION 3.16)
project(lingtu_rerun_stub LANGUAGES CXX)

add_library(rerun_sdk INTERFACE)
target_include_directories(rerun_sdk INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
