cmake_minimum_required(VERSION 3.20)
project(mls_limit_maps LANGUAGES CXX)
add_executable(make_scenes make_scenes.cpp)
target_compile_features(make_scenes PRIVATE cxx_std_17)
target_compile_definitions(make_scenes PRIVATE NOMINMAX)
target_include_directories(make_scenes PRIVATE C:/opt/octomap-1.10.0-msvc-x64/include)
target_link_libraries(make_scenes PRIVATE
  C:/opt/octomap-1.10.0-msvc-x64/lib/octomap.lib
  C:/opt/octomap-1.10.0-msvc-x64/lib/octomath.lib)
